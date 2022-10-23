# -*- coding: utf-8 -*-
"""
Common utility classes and functions.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     11.02.2022
@modified    23.10.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.util
import asyncio
import functools
import hashlib
import inspect
import logging
import math
import os
import re
import threading
import time



class ThrottledLogger(logging.Logger):
    """
    Logger wrapper with support for throttling logged messages per call site.

    Logging methods (`debug()`, `info()`, etc) accept additional keyword arguments:
    - `__once__`:                whether to log only once from call site
    - `__throttle__`:            seconds to skip logging from call site for
    - `__throttle_identical__`:  whether to skip logging identical consecutive texts from call site
                                 (given log message excluding formatting arguments).
                                 Combines with `__throttle__` to skip a duplicate for a period.
    """

    _KEYWORDS = ["__throttle__", "__throttle_identical__", "__once__"]

    ## Caller IDs registered for throttling by once-only
    _ONCES = set()

    ## Caller IDs and last timestamps for throttling by time
    _TIMES = {}

    ## Caller IDs and log message hashes for throttling by identical text
    _HASHES = {}


    def __init__(self, logger):
        """
        Creates a wrapper logger around given logger instance, providing support for throttling.

        @param   `logging.Logger` to wrap
        """
        super().__init__(logger.name, logger.level)
        OVERRIDES = ("debug", "info", "warning", "warn", "error", "fatal", "log")
        for name in dir(logger):  # Monkey-patch all other attributes from wrapped
            if not name.startswith("_") and name not in OVERRIDES:
                setattr(self, name, getattr(logger, name))
        self.__logger = logger


    def debug(self, msg, *args, **kwargs):
        """
        Logs `msg % args` with severity `DEBUG`.

        To pass exception information, use the keyword argument `exc_info=True`.

        @param   __once__                 whether to log only once from call site
        @param   __throttle__             seconds to skip logging from call site for
        @param   __throttle_identical__   whether to skip identical consecutive texts from call site
        """
        if not self._is_throttled(msg, **self._extract_args(kwargs)):
            self.__logger.debug(msg, *args, **kwargs)


    def info(self, msg, *args, **kwargs):
        """
        Logs `msg % args` with severity `INFO`. The arguments are interpreted as for `debug()`.
        """
        if not self._is_throttled(msg, **self._extract_args(kwargs)):
            self.__logger.info(msg, *args, **kwargs)


    def warning(self, msg, *args, **kwargs):
        """
        Logs `msg % args` with severity `WARN`. The arguments are interpreted as for `debug()`.
        """
        if not self._is_throttled(msg, **self._extract_args(kwargs)):
            self.__logger.warning(msg, *args, **kwargs)


    def warn(self, msg, *args, **kwargs):
        """
        Logs `msg % args` with severity `WARNING`. The arguments are interpreted as for `debug()`.
        """
        if not self._is_throttled(msg, **self._extract_args(kwargs)):
            self.__logger.warn(msg, *args, **kwargs)


    def error(self, msg, *args, **kwargs):
        """
        Logs `msg % args` with severity `ERROR`. The arguments are interpreted as for `debug()`.
        """
        if not self._is_throttled(msg, **self._extract_args(kwargs)):
            self.__logger.error(msg, *args, **kwargs)


    def fatal(self, msg, *args, **kwargs):
        """
        Logs `msg % args` with severity `FATAL`. The arguments are interpreted as for `debug()`.
        """
        if not self._is_throttled(msg, **self._extract_args(kwargs)):
            self.__logger.fatal(msg, *args, **kwargs)


    def log(self, level, msg, *args, **kwargs):
        """
        Logs `msg % args` with given severity. The arguments are interpreted as for `debug()`.
        """
        if not self._is_throttled(msg, **self._extract_args(kwargs)):
            self.__logger.log(level, msg, *args, **kwargs)


    @classmethod
    def _extract_args(cls, kwargs):
        """Drops throttle parameters from kwargs and returns as dict."""
        return {k.replace("__", ""): kwargs.pop(k, None) for k in cls._KEYWORDS}


    @classmethod
    def _is_throttled(cls, msg, once=False, throttle=None, throttle_identical=False):
        """Returns whether message should be skipped."""
        result = False
        if once or throttle_identical or throttle:
            f = inspect.currentframe().f_back.f_back
            caller_id = ":".join(str(x) for x in (inspect.getabsfile(f), f.f_lineno, f.f_lasti))
        if once:
            result = caller_id in cls._ONCES
            cls._ONCES.add(caller_id)
        elif throttle_identical:
            msg_hash = hashlib.md5(msg.encode()).hexdigest()
            result = (cls._HASHES.get(caller_id) == msg_hash)
            if not result:
                cls._HASHES[caller_id] = msg_hash
            if throttle:
                now, last = time.time(), cls._TIMES.get(caller_id)
                result = result and last is not None and now - last < throttle
                cls._TIMES[caller_id] = now
        elif throttle:
            now, last = time.time(), cls._TIMES.get(caller_id)
            result = last is not None and now - last < throttle
            cls._TIMES[caller_id] = now
        return result



def drop_zeros(v, replace=""):
    """Drops or replaces trailing zeros and empty decimal separator, if any."""
    return re.sub(r"\.?0+$", lambda x: len(x.group()) * replace, str(v))


def ensure_object(obj_or_cls, attributes, *args, **kwargs):
    """
    Ensures result is an object of specified type.

    Intended for wrapping convenience functions giving or returning
    object as attribute list or dictionary instead of instantiated object.

    If first positional argument is an instance of specified class,
    it is populated from positional and keyword arguments.

    `obj_or_cls` may be a class to instantiate, or the class instance
    to populate if instance not in first positional argument.
    Instance attributes will be given in constructor as keyword arguments.

    E.g. `ensure_object(std_msgs.msg.Bool, ["data"], True)`.

    @param   obj_or_cls  object class or instance
    @param   attributes  iterable of object attribute names
                         to combine positional arguments from,
                         or a callable returning iterable
    @return              object of specified class, populated from
                         positional and keyword arguments,
                         created if not given
    """
    iscls = inspect.isclass(obj_or_cls)
    cls, obj = (obj_or_cls if iscls else type(obj_or_cls)), None
    isarg = args and isinstance(args[0], cls)  # Object in first positional arg
    if isarg: obj, args = (args[0], args[1:])
    if callable(attributes): attributes = attributes()
    for k, v in zip(attributes, args):
        if k in kwargs:
            raise TypeError("%s got multiple values for keyword argument %r" %
                            (cls.__name__, k))
        kwargs[k] = v
    obj = obj or (cls(**kwargs) if iscls else obj_or_cls)
    for k, v in kwargs.items() if isarg or not iscls else ():
        setattr(obj, k, v)
    return obj


def flatten_dict(dct, sep="."):
    """
    Flattens a nested dictionary to a flat dictionary, with nested keys joined with separator.

    @param   dct  the dictionary to flatten
    @param   sep  separator between nested keys
    @return       flat dictionary like {"my.nested.key": value}
    """
    result, stack = {}, list(dct.items())
    while stack:
        k, v = stack.pop(0)
        if isinstance(v, dict):
            stack.extend(("%s%s%s" % (k, sep, k2), v2) for k2, v2 in v.items())
        else:
            result[k] = v
    return result


def format_bytes(size, precision=2, inter=" ", strip=True):
    """Returns a formatted byte size (like 421.40 MB), trailing zeros optionally removed."""
    result = "0 bytes"
    if size:
        UNITS = [("bytes", "byte")[1 == size]] + [x + "B" for x in "KMGTPEZY"]
        exponent = min(int(math.log(size, 1024)), len(UNITS) - 1)
        result = "%.*f" % (precision, size / (1024. ** exponent))
        result += "" if precision > 0 else "."  # Do not strip integer zeroes
        result = (drop_zeros(result) if strip else result) + inter + UNITS[exponent]
    return result


def get_arity(func, positional=True, keyword=False):
    """Returns the maximum number of arguments the function takes, -1 if variable number."""
    POSITIONALS = (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.POSITIONAL_ONLY)
    KEYWORDALS  = (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY)
    result, params = 0, inspect.signature(func).parameters
    if positional and any(x.kind == inspect.Parameter.VAR_POSITIONAL for x in params.values()) \
    or keyword    and any(x.kind == inspect.Parameter.VAR_KEYWORD    for x in params.values()):
        result = -1
    else:
        if positional:
            result += sum(x.kind in POSITIONALS for x in params.values())
        if keyword:
            result += sum(x.kind in KEYWORDALS  for x in params.values())
    return result


def get_nested(obj, path):
    """
    Returns (nested value, value parent container, key in parent container).

    Raises if path not found.

    @param   obj      object or dictionary or list, with arbitrary nesting
    @param   path     "name" or ("nested", "path") or "nested.path", element can be list index
    @return           (value, value parent container, key in parent container)
    """

    def getter(obj, key):
        """Returns dictionary value, or object attribute, or sequence element if numeric key."""
        key = int(key) if isinstance(key, str) and re.match("^[0-9]+$", key) else key
        return obj[key] if isinstance(obj, dict) or isinstance(key, int) else getattr(obj, key)

    (*path, leaf) = path.split(".") if isinstance(path, str) else path
    ptr = obj
    for p in path:
        ptr = getter(ptr, p)
    return getter(ptr, leaf), ptr, leaf


def get_value(obj, path, pathsep=None):
    """
    Returns object or dictionary or list value at (nested, path).

    Raises if path not found.

    @param   obj      object or dictionary or list, with arbitrary nesting
    @param   path     ("nested", "path") or "nested.path", element can be list index
    @param   pathsep  string to split scalar key by, like "." for "nested.path"
    """
    if not isinstance(path, (list, tuple)):
        path = path.split(pathsep) if pathsep else [path]
    path, leaf, ptr = path[:-1], path[-1], obj
    for p in path:
        ptr = ptr[p] if isinstance(ptr, (dict, list, tuple)) else getattr(ptr, p)

    if isinstance(ptr, (dict, list, tuple)):
        return ptr[leaf]
    return getattr(ptr, leaf)


def memoize(func):
    """
    Returns a results-caching wrapper for the function.

    All arguments to function must be hashable.
    """
    cache = {}
    def inner(*args, **kwargs):
        key = args + sum(kwargs.items(), ())
        if key not in cache:
            cache[key] = func(*args, **kwargs)
        return cache[key]
    return functools.update_wrapper(inner, func)


def make_dict(path, value):
    """Returns a nested dictionary from path, like {"nested": {"path": value}}."""
    result = ptr = {}
    for p in path[:-1]:
        ptr = ptr.setdefault(p, {})
    ptr[path[-1]] = value
    return result


def merge_dicts(d1, d2):
    """
    Merges d2 into d1, recursively for nested dicts.

    @return  updated d1
    """
    for k, v in d2.items() if d2 else ():
        if k in d1 and isinstance(v, dict) and isinstance(d1[k], dict):
            merge_dicts(d1[k], v)
        else:
            d1[k] = v
    return d1


def namejoin(*args):
    """Returns arguments joined into a namespace name, starting and separated with "/"."""
    return "/" + "/".join(filter(bool, (x.strip("/") for x in args)))


def namesplit(name):
    """Returns argument split into (namespace, name), like "/a/b/c" as ("/a/b", "c")."""
    parts = name.rsplit("/", 1)
    return ("" if len(parts) < 2 else (parts[0] or "/")), parts[-1]


def set_value(obj, path, value, pathsep=None):
    """
    Sets object or dictionary or list value at key or (nested, path).

    Lists are appended to if specified index does not exist.

    @param   obj      object or dictionary or list, with arbitrary nesting
    @param   path     scalar key or ("nested", "path"), element can be list index
    @param   value    value to set
    @param   pathsep  string to split scalar key by, like "." for "nested.path"
    """
    if not isinstance(path, (list, tuple)):
        path = path.split(pathsep) if pathsep else [path]
    path, leaf, ptr = path[:-1], path[-1], obj
    for p in path:
        ptr = ptr.setdefault(p, {}) if isinstance(ptr, dict) else \
              ptr[p] if isinstance(ptr, list) else getattr(ptr, p)

    if isinstance(ptr, dict):
        ptr[leaf] = value
    elif isinstance(ptr, list):
        if 0 <= leaf < len(ptr):
            ptr[leaf] = value
        else:
            ptr.append(value)
    else:
        setattr(ptr, leaf, value)


def start_future(func, *args, **kwargs):
    """
    Returns `asyncio.Future` and invokes function in a background thread.

    Future will be done when function returns or raises.

    Future is not executed with asyncio. Background thread is not daemonic.
    """
    future = asyncio.Future()
    def worker():
        try: future.set_result(func(*args, **kwargs))
        except Exception as e: future.set_exception(e)
    threading.Thread(target=worker).start()
    return future


def unique_path(pathname, empty_ok=False):
    """
    Returns a unique version of the path.

    If a file or directory with the same name already exists, returns a unique
    version (e.g. "/tmp/my.2.file" if ""/tmp/my.file" already exists).

    @param   empty_ok  whether to ignore existence if file is empty
    """
    result = pathname
    if os.path.isfile(result) and empty_ok and not os.path.getsize(result):
        return result
    path, name = os.path.split(result)
    base, ext = os.path.splitext(name)
    if len(name) > 255: # Filesystem limitation
        name = base[:255 - len(ext) - 2] + ".." + ext
        result = os.path.join(path, name)
    counter = 2
    while os.path.exists(result):
        suffix = ".%s%s" % (counter, ext)
        name = base + suffix
        if len(name) > 255:
            name = base[:255 - len(suffix) - 2] + ".." + suffix
        result = os.path.join(path, name)
        counter += 1
    return result


def wrap_arity(func):
    """
    Returns wrapper for invoking function with its maximum supported number of arguments.

    E.g. `wrap_arity(abs)(-1, -2)` will return result of `abs(-1)`.
    """
    POSITIONALS = (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.POSITIONAL_ONLY)
    KEYWORDALS  = (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY)
    params   = inspect.signature(func).parameters
    varargs  = any(x.kind == inspect.Parameter.VAR_POSITIONAL for x in params.values())
    varkws   = any(x.kind == inspect.Parameter.VAR_KEYWORD    for x in params.values())
    posargs  = [x.name for x in params.values() if x.kind in POSITIONALS]
    keywords = [x.name for x in params.values() if x.kind in KEYWORDALS]

    def inner(*args, **kwargs):
        if not varargs: args   = args[:len(posargs)]
        if not varkws:  kwargs = {k: v for k, v in kwargs.items() if k in keywords}
        return func(*args, **kwargs)
    return functools.update_wrapper(inner, func)


__all__ = [
    "ensure_object", "flatten_dict", "format_bytes", "get_arity", "get_value", "make_dict",
    "memoize", "merge_dicts", "namejoin", "namesplit", "set_value", "start_future",
    "unique_path", "wrap_arity",
]
