# -*- coding: utf-8 -*-
"""
Utilities for ROS built-in types, and message and service types.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     11.02.2022
@modified    26.10.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.api
import datetime
import decimal
import inspect
import os

ros1 = ros2 = None
if os.getenv("ROS_VERSION") == "2":
    from . import ros2
else:
    from . import ros1
ros = ros1 or ros2


## All built-in numeric types in ROS
ROS_NUMERIC_TYPES = ["byte", "char", "int8", "int16", "int32", "int64", "uint8",
                     "uint16", "uint32", "uint64", "float32", "float64", "bool"]

## All built-in string types in ROS
ROS_STRING_TYPES = ["string", "wstring"]

## All built-in basic types in ROS
ROS_BUILTIN_TYPES = ROS_NUMERIC_TYPES + ROS_STRING_TYPES

## Python constructors for ROS built-in types, as {ROS name: type class}
ROS_BUILTIN_CTORS = {"byte":   int,  "char":   int, "int8":    int,   "int16":   int,
                     "int32":  int,  "int64":  int, "uint8":   int,   "uint16":  int,
                     "uint32": int,  "uint64": int, "float32": float, "float64": float,
                     "bool":   bool, "string": str, "wstring": str}

## ROS time/duration types
ROS_TIME_TYPES = ros.ROS_TIME_TYPES

## ROS time/duration types mapped to type names
ROS_TIME_CLASSES = ros.ROS_TIME_CLASSES

## All built-in basic types plus time types in ROS
ROS_COMMON_TYPES = ROS_BUILTIN_TYPES + ROS_TIME_TYPES

## Mapping between type aliases and real types, like {"byte": "int8"} in ROS1
ROS_ALIAS_TYPES = ros.ROS_ALIAS_TYPES

## Separator char between ROS parameter namespace parts
PARAM_SEPARATOR = ros.PARAM_SEPARATOR

## Prefix for "private" names, auto-namespaced under current namespace
PRIVATE_PREFIX = ros.PRIVATE_PREFIX

## ROS Python module family, "rospy" or "rclpy"
FAMILY = ros.FAMILY

## Logging handler that forwards logging messages to ROS logger
ROSLogHandler = ros.ROSLogHandler


def canonical(typename):
    """Returns "pkg/Type" for "pkg/subdir/Type"."""
    return ros.canonical(typename)


def format_param_name(name):
    """Returns parameter name with correct separator for ROS version, and leading sigils stripped."""
    return ros.format_param_name(name)


def get_message_class(msg_or_type):
    """
    Returns ROS message / service class object.

    @param   msg_or_type  full or canonical class name,
                          like "std_msgs/Bool" or "std_srvs/SetBool" or "std_srvs/SetBoolRequest";
                          or class instance like `std_msgs.msg.Bool()`
    @return               ROS message / service class object, like `std_msgs.msg.Bool`
                          or `std_srvs.srv.SetBool` or `std_srvs.srv.SetBoolRequest`,
                          or None if not found
    """
    return ros.get_message_class(msg_or_type)


def get_message_definition(msg_or_type, full=True):
    """
    Returns ROS message or service request/response type definition text.

    Text will include subtype definitions by default.

    @param   msg_or_type  canonical or full class name like "std_msgs/Bool" or "std_msgs/msg/Bool",
                          or class instance like `std_msgs.msg.Bool()`,
                          or class object like `std_msgs.msg.Bool`
    @param   full         include definitions of nested types, separated by "\n---\nMSG: pkg/Type\n"
                          (ignored for service request/response types)
    @return               message type definition text
    """
    return ros.get_message_definition(msg_or_type, full)


def get_message_fields(val):
    """
    Returns {field name: field type name} if ROS message or service request/response, else {}.

    @param   val  ROS message or service request/response instance, or class object
    """
    return ros.get_message_fields(val)


def get_message_header(val):
    """
    Returns message `std_msgs/Header`-attribute if any, else `None`.

    @param   val  ROS message or service request/response instance
    """
    return ros.get_message_header(val)


def get_message_type(msg_or_cls):
    """
    Returns ROS message / service canonical type name, like "std_msgs/Header".

    Returns "*" for `AnyMsg`.

    @param   msg_or_cls  class instance like `std_msgs.msg.Bool()`,
                         or class object like `std_msgs.msg.Bool`
    @return   canonical name, or `None` if not ROS message / service
    """
    return ros.get_message_type(msg_or_cls)


def get_message_type_hash(msg_or_type):
    """
    Returns ROS message / service type MD5 hash.

    @param   msg_or_type  full or canonical class name
                          like "std_msgs/Bool" or "std_srvs/SetBool" or "std_srvs/SetBoolRequest",
                          or class instance like `std_msgs.msg.Bool()`,
                          or class object like `std_msgs.msg.Bool`
    """
    return ros.get_message_type_hash(msg_or_type)


def get_message_value(msg, name):
    """
    Returns object attribute value, with numeric arrays converted to lists.

    @param   message attribute name; may also be (nested, path) or "nested.path"
    """
    return ros.get_message_value(msg, name)


def get_service_definition(srv_or_type):
    """
    Returns ROS service type definition text.

    @param   srv_or_type  canonical or full class name
                          like "std_srvs/SetBool" or "std_srvs/srv/SetBool",
                          or class instance like `std_srvs.srv.SetBool()`,
                          or class object like `std_srvs.srv.SetBool`
    @return               ROS service type definition text
    """
    return ros.get_service_definition(srv_or_type)


def get_service_request_class(srv_or_type):
    """
    Returns ROS service request class object.

    @param   srv_or_type  canonical or full class name
                          like "std_srvs/SetBool" or "std_srvs/srv/SetBool",
                          or class instance like `std_srvs.srv.SetBool()`,
                          or class object like `std_srvs.srv.SetBool`
    @return               ROS service request class object, like `std_srvs.srv.SetBoolRequest`
    """
    return ros.get_service_request_class(srv_or_type)


def get_service_response_class(srv_or_type):
    """
    Returns ROS service response class object.

    @param   srv_or_type  canonical or full class name
                          like "std_srvs/SetBool" or "std_srvs/srv/SetBool",
                          or class instance like `std_srvs.srv.SetBool()`,
                          or class object like `std_srvs.srv.SetBool`
    @return               ROS service response class object, like `std_srvs.srv.SetBoolResponse`
    """
    return ros.get_service_response_class(srv_or_type)


def get_type_alias(typename):
    """
    Returns alias like "char" for ROS built-in type, if any; reverse of get_alias_type().

    In ROS1, byte and char are aliases for int8 and uint8; in ROS2 the reverse.
    """
    return next((k for k, v in ROS_ALIAS_TYPES.items() if v == typename), None)


def get_alias_type(typename):
    """
    Returns ROS built-in type for alias like "char", if any; reverse of get_type_alias().

    In ROS1, byte and char are aliases for int8 and uint8; in ROS2 the reverse.
    """
    return ROS_ALIAS_TYPES.get(typename)


def is_ros_message(val):
    """
    Returns whether value is a ROS message or service request/response class or instance.

    @param   val  like `std_msgs.msg.Bool()` or `std_srvs.srv.SetBoolRequest`
    @return       True if value is a ROS message or service request/response class or instance,
                  False otherwise
    """
    return ros.is_ros_message(val)


def is_ros_service(val):
    """Returns whether value is a ROS service class object."""
    return ros.is_ros_service(val)


def is_ros_time(val):
    """Returns whether value is a ROS time/duration class or instance."""
    return ros.is_ros_time(val)


def make_duration(secs=0, nsecs=0):
    """Returns a ROS duration."""
    return ros.make_duration(secs=secs, nsecs=nsecs)


def make_time(secs=0, nsecs=0):
    """Returns a ROS time."""
    return ros.make_time(secs=secs, nsecs=nsecs)


def make_full_typename(typename, category="msg"):
    """
    Returns "pkg/msg/Type" for "pkg/Type".

    @param   category  type category like "msg" or "srv"
    """
    INTER = "/%s/" % category
    if INTER in typename or "/" not in typename or typename.startswith("%s/" % FAMILY):
        return typename
    return INTER.join(next((x[0], x[-1]) for x in [typename.split("/")]))


def dict_to_message(dct, msg):
    """
    Returns given ROS message populated from Python dictionary.

    Raises TypeError on attribute value type mismatch.
    """
    for name, typename in ros.get_message_fields(msg).items():
        if name not in dct:
            continue  # for name,
        v, msgv = dct[name], ros.get_message_value(msg, name)

        if ros.is_ros_message(msgv):
            v = dict_to_message(v, msgv)
        elif isinstance(msgv, (list, tuple)):
            scalarname = ros.scalar(typename)
            if scalarname in ROS_BUILTIN_TYPES:
                cls = ROS_BUILTIN_CTORS[scalarname]
                v = [x if isinstance(x, cls) else cls(x) for x in v]
            else:
                cls = ros.get_message_class(scalarname)
                v = [dict_to_message(x, cls()) for x in v]
        else:
            v = type(msgv)(v)

        setattr(msg, name, v)
    return msg


def message_to_dict(msg, replace=None):
    """
    Returns ROS message as nested Python dictionary.

    @param   replace  mapping of {value: replaced value},
                      e.g. {math.nan: None, math.inf: None}
    """
    result = {} if ros.is_ros_message(msg) else msg
    for name, typename in ros.get_message_fields(msg).items():
        v = ros.get_message_value(msg, name)
        if ros.is_ros_time(v):
            v = dict(zip(get_message_fields(v), ros.to_sec_nsec(v)))
        elif ros.is_ros_message(v):
            v = message_to_dict(v)
        elif isinstance(v, (list, tuple)):
            if ros.scalar(typename) not in ROS_BUILTIN_TYPES:
                v = [message_to_dict(x) for x in v]
            elif replace:
                v = [replace.get(x, x) for x in v]
        elif replace:
            v = replace.get(v, v)
        result[name] = v
    return result


def message_to_str(msg, indent=None):
    """
    Returns ROS message as an evaluatable string, e.g. "std_msgs.msg.UInt8(data=0)".

    @param   indent  multi-line indentation level to use if not returning one-liner;
                     as the number of spaces or the string to indent with
    """
    if not is_ros_message(msg):
        return repr(msg)
    indent = " " * indent if isinstance(indent, int) else indent or ""
    sep, start, end = (",\n", "\n%s" % indent, "\n") if indent else (", ", "", "")
    parts = []
    for k in get_message_fields(msg):
        v = get_message_value(msg, k)
        if is_ros_message(v):
            v = message_to_str(v, indent)
            if indent:
                v = v.replace("\n", start)
        elif isinstance(v, bytes):
            v = "bytes(%s)" % list(v)
        elif isinstance(v, list) and v:
            nested = sep.join(message_to_str(x, indent) for x in v)
            if indent:
                nested = indent + nested.replace("\n", "\n%s" % (indent * 2))
            v = "[%s%s%s%s]" % (start, nested, end, indent)
        else:
            v = repr(v)
        parts.append("%s=%s" % (k, v))
    name = make_full_typename(get_message_type(msg)).replace("/", ".")
    return "%s(%s%s%s)" % (name, start, (sep + indent).join(parts), end)


def serialize_message(msg):
    """Returns ROS message or service request/response as a serialized binary of `bytes()`."""
    return ros.serialize_message(msg)


def deserialize_message(raw, cls_or_typename):
    """Returns ROS message or service request/response instantiated from serialized binary."""
    return ros.deserialize_message(raw, cls_or_typename)


def scalar(typename):
    """
    Returns scalar type from ROS message data type, like "uint8" from uint8-array.

    Returns type unchanged if an ordinary type. In ROS2, returns unbounded type,
    e.g. "string" from "string<=10[<=5]".
    """
    return ros.scalar(typename)


def time_category(msg_or_type):
    """
    Returns "time" or "duration" for time/duration type, else original value.

    @param   msg_or_type  full or canonical class name
                          like "duration" or "builtin_interfaces/Time",
                          or class instance like `rospy.Time()`,
                          or class object like `rclpy.time.Time`
    """
    typename = msg_or_type
    if not isinstance(typename, str):
        cls = msg_or_type if inspect.isclass(msg_or_type) else type(msg_or_type)
        typename = cls.__name__ if issubclass(cls, tuple(ROS_TIME_CLASSES)) else None
    return msg_or_type if not typename else "duration" if "duration" in typename.lower() else "time"


def time_message(val, to_message=True, clock_type=None):
    """
    Converts ROS2 time/duration between `rclpy` and `builtin_interfaces` objects.

    @param   val         ROS2 time/duration object from `rclpy` or `builtin_interfaces`
    @param   to_message  whether to convert from `rclpy` to `builtin_interfaces` or vice versa
    @param   clock_type  ClockType for converting to `rclpy.Time`, defaults to `ROS_TIME`
    @return              value converted to appropriate type, or original value if not convertible
    """
    if ros1: return val
    return ros2.time_message(val, to_message)


def to_datetime(val):
    """Returns value as datetime.datetime if value is ROS time/duration, else value."""
    sec = ros.to_sec(val)
    return datetime.datetime.fromtimestamp(sec) if sec is not val else val


def to_decimal(val):
    """Returns value as decimal.Decimal if value is ROS time/duration, else value."""
    if ros.is_ros_time(val) and not inspect.isclass(val):
        return decimal.Decimal("%d.%09d" % ros.to_sec_nsec(val))
    return val


def to_nsec(val):
    """Returns value in nanoseconds if value is ROS time/duration, else value."""
    return ros.to_nsec(val)


def to_sec(val):
    """Returns value in seconds if value is ROS time/duration, else value."""
    return ros.to_sec(val)


def to_sec_nsec(val):
    """Returns value as (seconds, nanoseconds) if value is ROS time/duration, else value."""
    return ros.to_sec_nsec(val)


def to_time(val):
    """Returns value as ROS time if convertible (int/float/duration/datetime/decimal), else value."""
    return ros.to_time(val)


__all__ = [
    "ROSLogHandler", "FAMILY", "PARAM_SEPARATOR", "PRIVATE_PREFIX", "ROS_ALIAS_TYPES",
    "ROS_BUILTIN_CTORS", "ROS_BUILTIN_TYPES", "ROS_COMMON_TYPES", "ROS_NUMERIC_TYPES",
    "ROS_STRING_TYPES", "ROS_TIME_CLASSES", "ROS_TIME_TYPES", "canonical",
    "deserialize_message", "dict_to_message", "format_param_name", "get_alias_type",
    "get_message_class", "get_message_definition", "get_message_fields",
    "get_message_type", "get_message_type_hash", "get_message_value",
    "get_ros_time_category", "get_service_definition", "get_service_request_class",
    "get_service_response_class", "get_type_alias", "is_ros_message", "is_ros_service",
    "is_ros_time", "make_duration", "make_full_typename", "make_time", "message_to_dict",
    "scalar", "serialize_message", "time_message", "to_datetime", "to_decimal", "to_nsec",
    "to_sec", "to_sec_nsec", "to_time"
]
