# -*- coding: utf-8 -*-
"""
Utilities for ROS message definition texts.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     12.02.2022
@modified    14.04.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.parsing
import hashlib
import re

try:  # ROS2 only
    import rosidl_parser.parser
    import rosidl_parser.definition
    import rosidl_runtime_py
except ImportError: pass


# from . import api  # Imported late to avoid circular import
from . util import memoize



@memoize
def calculate_definition_hash(typename, definition, subdefs=()):
    """
    Returns MD5 hash for message / service type definition.

    @param   subdefs  definitions for subtypes, if any, as ((typename, definition), )
    """
    pkg, parts = typename.rsplit("/", 1)[0], definition.split("\n---\n", 1)
    subdefs = (subdefs or ()) + tuple(parse_definition_subtypes(definition).items())
    texts = [make_definition_hash_text(pkg, x, subdefs) for x in parts]
    return hashlib.md5("".join(texts).encode()).hexdigest()


@memoize
def make_definition_hash_text(pkg, definition, subdefs=()):
    """
    Returns text for calculating message or service request/response type definition.

    @param   subdefs  definitions for subtypes, as ((typename, definition), )
    """
    from . import api  # Imported late to avoid circular import
    # "type name (= constvalue)?" or "type name (defaultvalue)?" (ROS2 format)
    FIELD_RGX = re.compile(r"^([a-z][^\s:]+)\s+([^\s=]+)(\s*=\s*([^\n]+))?(\s+([^\n]+))?", re.I)
    STR_CONST_RGX = re.compile(r"^w?string\s+([^\s=#]+)\s*=")
    lines, subdefmap = [], dict(subdefs or ())

    # First pass: collect constants
    for line in definition.splitlines():
        if set(line) == set("="):  # Subtype separator
            break  # for line
        # String constants cannot have line comments
        if "#" in line and not STR_CONST_RGX.match(line): line = line[:line.index("#")]
        match = FIELD_RGX.match(line)
        if match and match.group(3):
            lines.append("%s %s=%s" % (match.group(1), match.group(2), match.group(4).strip()))
    # Second pass: collect fields and subtype hashes
    for line in definition.splitlines():
        if set(line) == set("="):  # Subtype separator
            break  # for line
        if "#" in line and not STR_CONST_RGX.match(line): line = line[:line.index("#")]
        match = FIELD_RGX.match(line)
        if match and not match.group(3):  # Not constant
            scalartype, namestr = api.scalar(match.group(1)), match.group(2)
            if scalartype in api.ROS_COMMON_TYPES:
                typestr = match.group(1)
                if match.group(5): namestr = (namestr + " " + match.group(6)).strip()
            else:
                subtype = scalartype if "/" in scalartype else "std_msgs/Header" \
                          if "Header" == scalartype else "%s/%s" % (pkg, scalartype)
                typestr = calculate_definition_hash(subtype, subdefmap[subtype], subdefs)
            lines.append("%s %s" % (typestr, namestr))
    return "\n".join(lines).strip()


@memoize
def parse_definition_subtypes(typedef):
    """
    Returns subtype names and type definitions from a full message definition.

    @return  {"pkg/MsgType": "full definition for MsgType including subtypes"}
    """
    from . import api  # Imported late to avoid circular import
    result = {}  # {subtypename: subtypedef}
    curtype, curlines = "", []
    rgx = re.compile(r"^((=+)|(MSG: (.+)))$")  # Group 2: separator, 4: new type
    for line in typedef.splitlines():
        m = rgx.match(line)
        if m and m.group(2) and curtype:  # Separator line between nested definitions
            result[curtype] = "\n".join(curlines)
            curtype, curlines = "", []
        elif m and m.group(4):  # Start of nested definition "MSG: pkg/MsgType"
            curtype, curlines = m.group(4), []
        elif not m and curtype:  # Nested definition content
            curlines.append(line)
    if curtype:
        result[curtype] = "\n".join(curlines)

    # "type name (= constvalue)?" or "type name (defaultvalue)?" (ROS2 format)
    FIELD_RGX = re.compile(r"^([a-z][^\s]+)\s+([^\s=]+)(\s*=\s*([^\n]+))?(\s+([^\n]+))?", re.I)
    for subtype, subdef in list(result.items()):
        pkg = subtype.rsplit("/", 1)[0]
        for line in subdef.splitlines():
            m = FIELD_RGX.match(line)
            if m and m.group(1):
                scalartype, fulltype = api.scalar(m.group(1)), None
                if scalartype not in api.ROS_COMMON_TYPES:
                    fulltype = scalartype if "/" in scalartype else "std_msgs/Header" \
                               if "Header" == scalartype else "%s/%s" % (pkg, scalartype)
                if fulltype in result:
                    addendum = "%s\nMSG: %s\n%s" % ("=" * 80, fulltype, result[fulltype])
                    result[subtype] = result[subtype].rstrip() + ("\n\n%s\n" % addendum)
    return result



@memoize
def get_ros2_message_definition(typename, full=True):
    """
    Returns ROS2 message/service type definition full text.

    Parses and assembles text from .msg or .srv or .idl files on disk.

    @param   full  include subtype definitions, separated with lines of "="
    """
    from . import api  # Imported late to avoid circular import
    texts, pkg = {}, typename.rsplit("/", 1)[0]
    category = "srv" if "/srv/" in typename else "msg"
    if "srv" == category: full = False
    try:
        basepath = api.make_full_typename(typename) + (".%s" % category)
        with open(rosidl_runtime_py.get_interface_path(basepath)) as f:
            texts[typename] = f.read()
    except Exception:  # .msg/.srv file unavailable: parse IDL
        texts[typename] = get_ros2_message_definition_idl(typename)
    for line in texts[typename].splitlines() if full else ():
        if not line or not line[0].isalpha():
            continue  # for line
        linetype = api.scalar(api.canonical(re.sub(r"^([a-zA-Z][^\s]+)(.+)", r"\1", line)))
        if linetype in api.ROS_BUILTIN_TYPES:
            continue  # for line
        linetype = linetype if "/" in linetype else "std_msgs/Header" \
                   if "Header" == linetype else "%s/%s" % (pkg, linetype)
        linedef = None if linetype in texts else get_ros2_message_definition(linetype)
        if linedef: texts[linetype] = linedef

    basedef = texts.pop(next(iter(texts)))
    subdefs = ["%s\nMSG: %s\n%s" % ("=" * 80, k, v) for k, v in texts.items()]
    return basedef + ("\n" if subdefs else "") + "\n".join(subdefs)


@memoize
def get_ros2_service_definition(typename):
    """
    Returns ROS2 service type definition full text.

    Parses and assembles text from .srv or .idl files on disk.
    """
    from . import api  # Imported late to avoid circular import
    text = None
    try:
        basepath = api.make_full_typename(typename, "srv") + ".srv"
        with open(rosidl_runtime_py.get_interface_path(basepath)) as f:
            text = f.read()
    except Exception:  # .srv file unavailable: parse IDL
        text = get_ros2_service_definition_idl(typename)
    return text


@memoize
def get_ros2_message_definition_idl(typename):
    """Returns ROS2 message type definition parsed from IDL file."""
    from . import api  # Imported late to avoid circular import

    basepath = api.make_full_typename(typename) + ".idl"
    typepath = rosidl_runtime_py.get_interface_path(basepath)
    with open(typepath) as f:
        idlcontent = rosidl_parser.parser.parse_idl_string(f.read())
    msgidl = idlcontent.get_elements_of_type(rosidl_parser.definition.Message)[0]
    return rosidl_format_message_content(msgidl)


@memoize
def get_ros2_service_definition_idl(typename):
    """Returns ROS2 service type definition parsed from IDL file."""
    from . import api  # Imported late to avoid circular import

    basepath = api.make_full_typename(typename, "srv") + ".idl"
    typepath = rosidl_runtime_py.get_interface_path(basepath)
    with open(typepath) as f:
        idlcontent = rosidl_parser.parser.parse_idl_string(f.read())

    srvidl = idlcontent.get_elements_of_type(rosidl_parser.definition.Service)[0]
    msgidls = (srvidl.request_message, srvidl.response_message)
    return "\n---\n".join(map(rosidl_format_message_content, msgidls)).lstrip()


def rosidl_format_idl_type(typeobj, msgpackage, constant=False):
    """
    Returns canonical type name for ROS2 IDL parser entity, like "uint8" or "nav_msgs/Path".

    @param   typeobj     ROS2 IDL parser entity, like `rosidl_parser.definition.Array`
    @param   msgpackage  name of parsed package
    @param   constant    whether parsed item is a constant
    """
    from . import api, ros2  # Imported late to avoid circular import
    result = None
    if isinstance(typeobj, rosidl_parser.definition.AbstractNestedType):
        # Array, BoundedSequence, UnboundedSequence
        valuetype = rosidl_format_idl_type(typeobj.value_type, msgpackage, constant)
        size, bounding = "", ""
        if isinstance(typeobj, rosidl_parser.definition.Array):
            size = typeobj.size
        elif typeobj.has_maximum_size():
            size = typeobj.maximum_size
        if isinstance(typeobj, rosidl_parser.definition.BoundedSequence):
            bounding = "<="
        result = "%s[%s%s]" % (valuetype, bounding, size) # type[], type[N], type[<=N]
    elif isinstance(typeobj, rosidl_parser.definition.AbstractWString):
        result = "wstring"
    elif isinstance(typeobj, rosidl_parser.definition.AbstractString):
        result = "string"
    elif isinstance(typeobj, rosidl_parser.definition.NamespacedType):
        nameparts = typeobj.namespaced_name()
        result = api.canonical("/".join(nameparts))
        if nameparts[0].value == msgpackage or "std_msgs/Header" == result:
            result = api.canonical("/".join(nameparts[-1:]))  # Omit package if local or Header
    else:  # Primitive like int8
        result = ros2.DDS_TYPES.get(typeobj.typename, typeobj.typename)

    if isinstance(typeobj, rosidl_parser.definition.AbstractGenericString) \
    and typeobj.has_maximum_size() and not constant:  # Constants get parsed into "string<=N"
        result += "<=%s" % typeobj.maximum_size

    return result


def rosidl_format_comment(text):
    """Returns annotation text formatted with comment prefixes and escapes."""
    ESCAPES = {"\n":   "\\n", "\t":   "\\t", "\x07": "\\a",
               "\x08": "\\b", "\x0b": "\\v", "\x0c": "\\f"}
    repl = lambda m: ESCAPES[m.group(0)]
    return "#" + "\n#".join(re.sub("|".join(map(re.escape, ESCAPES)), repl, l)
                            for l in text.split("\\n"))


def rosidl_get_comments(obj):
    """Returns all comments for annotatable object, as [text, ]."""
    return [v.get("text", "") for v in obj.get_annotation_values("verbatim")
            if "comment" == v.get("language")]


def rosidl_format_message_content(msgidl):
    """
    Returns message IDL as .msg text.

    @param   msgidl  rosidl_parser.definition.Message instance
    """
    lines = []
    package = msgidl.structure.namespaced_type.namespaces[0]
    DUMMY = rosidl_parser.definition.EMPTY_STRUCTURE_REQUIRED_MEMBER_NAME

    # Add general comments
    lines.extend(map(rosidl_format_comment, rosidl_get_comments(msgidl.structure)))
    # Add blank line between general comments and constants
    if lines and msgidl.constants: lines.append("")
    # Add constants
    for c in msgidl.constants:
        ctype = rosidl_format_idl_type(c.type, package, constant=True)
        lines.extend(map(rosidl_format_comment, rosidl_get_comments(c)))
        lines.append("%s %s=%s" % (ctype, c.name, c.value))
    # (Parser adds dummy placeholder if constants-only message)
    if not (len(msgidl.structure.members) == 1 and DUMMY == msgidl.structure.members[0].name):
        # Add blank line between constants and fields
        if msgidl.constants and msgidl.structure.members: lines.append("")
        # Add fields
        for m in msgidl.structure.members:
            lines.extend(map(rosidl_format_comment, rosidl_get_comments(m)))
            lines.append("%s %s" % (rosidl_format_idl_type(m.type, package), m.name))
    return "\n".join(lines)


__all__ = [
    "calculate_definition_hash", "get_ros2_message_definition", "get_ros2_service_definition"
]
