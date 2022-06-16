# -*- coding: utf-8 -*-
"""
Common utility functions.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     11.02.2022
@modified    12.04.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.util
import asyncio
import functools
import inspect
import threading



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


def memoize(func):
    """Returns a results-caching wrapper for the function."""
    cache = {}
    def inner(*args, **kwargs):
        key = args + sum(kwargs.items(), ())
        if key not in cache:
            cache[key] = func(*args, **kwargs)
        return cache[key]
    return functools.update_wrapper(inner, func)


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

    Future is not executed with asyncio.
    """
    future = asyncio.Future()
    def worker():
        try: future.set_result(func(*args, **kwargs))
        except Exception as e: future.set_exception(e)
    threading.Thread(target=worker).start()
    return future


def wrap_arity(func):
    """
    Returns wrapper for invoking function with its maximum supported number of arguments.

    E.g. `wrap_arity(abs)(-1, -2)` will return result of `abs(-1)`.
    """
    def inner(*args, **kwargs):
        spec = inspect.getfullargspec(func)
        if spec.varargs is None: args = args[:len(spec.args)]
        if spec.varkw is None:
            kwargs = {k: v for k, v in kwargs.items() if k in spec.kwonlyargs}
        return func(*args, **kwargs)
    return functools.update_wrapper(inner, func)


__all__ = [
    "ensure_object", "flatten_dict", "get_value", "make_dict", "memoize", "merge_dicts",
    "namejoin", "namesplit", "set_value", "start_future", "wrap_arity",
]
