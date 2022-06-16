# -*- coding: utf-8 -*-
"""
ROS1 interface.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     11.02.2022
@modified    30.05.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.ros1
import inspect
import io
import logging
import re
import threading
import time
import traceback

import genpy
import roslib
import rospy
import rospy.core
import rospy.rostime
import rosservice

from . import patch
from . import util


## ROS1 time/duration types
ROS_TIME_TYPES = ["time", "duration"]

## ROS1 time/duration types mapped to type names
ROS_TIME_CLASSES = {rospy.Time: "time", rospy.Duration: "duration"}

## Mapping between ROS type aliases and real types, like {"byte": "int8"}
ROS_ALIAS_TYPES = {"byte": "int8", "char": "uint8"}

## Separator char between ROS1 parameter namespace parts
PARAM_SEPARATOR = "/"

## Prefix for "private" names, auto-namespaced under current namespace
PRIVATE_PREFIX = "~"

## rospy.MasterProxy instance
MASTER = None

## `threading.Thread` for start_spin()
SPINNER = None


logger = logging.getLogger(__name__)


class ROSLogHandler(logging.Handler):
    """Logging handler that forwards logging messages to ROS1 logger."""

    def __init__(self):
        super().__init__()
        self._logger = logging.getLogger("rosout")  # logging.Logger instance


    def emit(self, record):
        """Adds message to ROS1 logger."""
        if not self._logger.isEnabledFor(record.levelno):
            return
        try:
            text = record.msg % record.args if record.args else record.msg
        except Exception:
            text = record.msg
        if record.exc_info:
            text += "\n\n" + "".join(traceback.format_exception(*record.exc_info))
        self._logger.log(record.levelno, text)



class Mutex:
    """Container for local mutexes."""

    ## Mutex for `spin()`
    SPIN      = threading.RLock()

    ## Mutex for `start_spin()`
    SPIN_START = threading.RLock()


def init_node(name, args=None):
    """
    Initializes rospy and creates ROS1 node.

    @param   args  list of command-line arguments for the node
    """
    global MASTER, logger
    if MASTER: return

    patch.patch_ros1()
    logger.debug("Initializing ROS node %r.", name)
    rospy.init_node(name, args, disable_signals=True)
    MASTER = rospy.client.get_master()
    if not any(isinstance(x, ROSLogHandler) for x in logger.handlers):
        logger.addHandler(ROSLogHandler())


def register_init():
    """Informs `rosros` of ROS1 having been initialized outside of `init_node()`."""
    global MASTER
    if MASTER: return

    patch.patch_ros1()
    MASTER = rospy.client.get_master()
    if not any(isinstance(x, ROSLogHandler) for x in logger.handlers):
        logger.addHandler(ROSLogHandler())


def init_params(defaults=None, **defaultkws):
    """
    Declares all parameters on node from defaults dictionary.

    @param   defaults    nested dictionary with suitable keys and values for ROS
                         (keys must be valid namespace names,
                          list values must not contain nested values)
    @param   defaultkws  parameters as key=value
    @return              full nested parameters dictionary, combined from given defaults
                         and externally set parameters on the node
    """
    result = {}
    stack = list(util.merge_dicts(defaults or {}, defaultkws).items())
    while stack:
        k, v = stack.pop()
        if isinstance(v, dict):
            if not v:
                util.set_value(result, k.split(PARAM_SEPARATOR), v)
            stack.extend(("%s%s%s" % (k, PARAM_SEPARATOR, k2), v2) for k2, v2 in v.items())
        else:
            k = "~" + format_param_name(k)
            if not rospy.has_param(k):
                rospy.set_param(k, v)

    prefix = get_node_name().rstrip(PARAM_SEPARATOR) + PARAM_SEPARATOR
    for path in rospy.get_param_names():
        if path.startswith(prefix):
            path = path[len(prefix):]
            util.set_value(result, [x for x in path.split(PARAM_SEPARATOR) if x],
                           rospy.get_param("~" + path))
    logger.debug("Initialized node parameters %r.", result)
    return result


def has_param(name):
    """
    Returns whether the parameter exists.

    @param   name  full name of the parameter in node namespace
    """
    return rospy.has_param("~" + format_param_name(name))


def get_param(name, default=None, autoset=True):
    """
    Returns parameter value from current ROS1 node.

    @param   name      full name of the parameter in node namespace
    @param   default   optional default to return if parameter unknown
    @param   autoset   set default value to node parameter if unknown
    @return            parameter value, or default if parameter was unknown

    @throws  KeyError  if parameter not set and default not given
    """
    name = "~" + format_param_name(name)
    if autoset and default is not None and not rospy.has_param(name):
        rospy.set_param(name, default)
    return rospy.get_param(name, default)


def get_param_names():
    """Returns the names of all current ROS1 node parameters."""
    result = []
    prefix = get_node_name().rstrip(PARAM_SEPARATOR) + PARAM_SEPARATOR
    for path in rospy.get_param_names():
        if path.startswith(prefix): result.append(path[len(prefix):])
    return result


def get_params(nested=True):
    """
    Returns the current ROS1 node parameters, by default as nested dictionary.

    @param   nested  return a nested dictionary,
                     like `{"my": {"name": 1}}` vs {"my.name": 1}
    """
    result = {}
    prefix = get_node_name().rstrip(PARAM_SEPARATOR) + PARAM_SEPARATOR
    for path in rospy.get_param_names():
        if path.startswith(prefix):
            path = path[len(prefix):]
            key = [x for x in path.split(PARAM_SEPARATOR) if x] if nested else path
            util.set_value(result, key, rospy.get_param("~" + path))
    return result


def set_param(name, value):
    """
    Sets a parameter on the node.

    @param   name   full name of the parameter in node namespace
    @param   value  parameter value to set
    @return         the set value
    """
    rospy.set_param("~" + format_param_name(name), value)
    return value


def delete_param(name):
    """
    Deletes parameter from the node.

    @param   name      full name of the parameter in node namespace
    @throws  KeyError  if parameter not set
    """
    rospy.delete_param("~" + format_param_name(name))


def ok():
    """Returns whether ROS1 has been initialized and is not shut down."""
    return rospy.core.is_initialized() and \
           not (rospy.is_shutdown() or rospy.core.is_shutdown_requested())


def start_spin():
    """
    Sets ROS1 spinning forever in a background thread.

    Does nothing if spinning has already been started.
    """
    def do_spin():
        global SPINNER
        try:
            while SPINNER and ok(): spin_once(0.5)
        finally: SPINNER = None

    global SPINNER
    with Mutex.SPIN_START:
        if SPINNER or Mutex.SPIN._is_owned(): return
        SPINNER = threading.Thread(target=do_spin)
        SPINNER.start()


def spin():
    """Spins ROS1 forever."""
    global SPINNER

    if SPINNER:
        try: SPINNER.join()  # Wait on background thread instead
        except KeyboardInterrupt: SPINNER = None  # Signal thread to end
        return

    if Mutex.SPIN._is_owned():  # spin() started from another thread
        with Mutex.SPIN: return  # Wait on mutex instead
    with Mutex.SPIN:
        rospy.spin()


def spin_once(timeout=None):
    """
    Waits until timeout, or forever if timeout None or negative.

    @param  timeout  time to sleep, as seconds or ROS duration;
                     None or <0 waits forever
    """
    rospy.rostime.wallsleep(2**31 - 1 if timeout is None else to_sec(timeout))


def spin_until_future_complete(future, timeout=None):
    """
    Spins ROS1 until future complete or timeout reached or ROS shut down.

    @param  future   object with `asyncio.Future`-conforming interface to complete
                     (will not be awaited with `async`)
    @param  timeout  time to wait, as seconds or ROS1 duration
    """
    if future.done():
        return
    timeout = to_sec(timeout)
    if timeout == 0.0:
        if not future.done():
            spin_once(1E-9)
        return

    now = time.monotonic()
    deadline = (now + timeout) if timeout is not None and timeout >= 0 else -1
    while ok() and (deadline < 0 or now < deadline) and not future.done():
        spin_once(0.1)
        now = time.monotonic()
    if not future.done() and not ok():
        future.cancel("ROS shut down")


def shutdown():
    """Sends the shutdown signal to rospy."""
    rospy.signal_shutdown(reason="")


def create_client(service, cls_or_typename):
    """
    Returns a ROS1 service client instance, for invoking a service.

    @param   service          name of service to invoke
    @param   cls_or_typename  ROS1 service class object like `std_srvs.srv.SetBool`
                              or service type name like `"std_srvs/SetBool"
    @return                   `rospy.ServiceProxy`,
                              will have `call_async()` returning a future
    """
    return rospy.ServiceProxy(service, get_message_class(cls_or_typename) )


def create_service(service, cls_or_typename, callback):
    """
    Returns a ROS1 service server instance, for providing a service.

    @param   service          name of service to provide
    @param   cls_or_typename  ROS1 service class object like `std_srvs.srv.SetBool`
                              or service type name like `"std_srvs/SetBool"
    @param   callback         callback function, invoked with (req, resp) or (req)
    @return                   `rospy.Service`
    """
    return rospy.Service(service, get_message_class(cls_or_typename), util.wrap_arity(callback))


def create_publisher(topic, cls_or_typename, latch=False, queue_size=0):
    """
    Returns a ROS1 publisher instance.

    @param   topic            name of topic to open
    @param   cls_or_typename  ROS1 message class object like `std_msgs.msg.Bool`
                              or message type name like "std_msgs/Bool"
    @param   latch            provide last published message to new subscribers
    @param   queue_size       queue size of outgoing messages (0 or None: infinite)
    @return                   `rospy.Publisher`
    """
    cls = get_message_class(cls_or_typename)
    return rospy.Publisher(topic, cls, latch=latch, queue_size=queue_size or 0)


def create_subscriber(topic, cls_or_typename, callback, callback_args=None,
                      queue_size=0, raw=False):
    """
    Returns a ROS1 subscriber instance.

    @param   topic            name of topic to listen to
    @param   cls_or_typename  ROS1 message class object like `std_msgs.msg.Bool`
                              or message type name like "std_msgs/Bool"
    @param   callback         callback function, invoked with received message,
                              and with additional arguments if given
    @param   callback_args    additional arguments to pass to the callback, if any,
                              invoked as `callback(msg, callback_args)´
    @param   raw              make subscription and invoke callback with AnyMsg
    @param   queue_size       queue size of incoming messages (0 or None: infinite)
    @return                   `rospy.Subscriber`
    """
    cls = rospy.AnyMsg if raw else get_message_class(cls_or_typename)
    return rospy.Subscriber(topic, cls, callback, callback_args, queue_size=queue_size or None)


def create_timer(period, callback, oneshot=False, immediate=False):
    """
    Returns a ROS1 timer instance.

    @param   period     desired period between callbacks, as seconds or ROS1 duration
    @param   callback   callback function invoked on timer, with no arguments
    @param   oneshot    whether to fire only once
    @param   immediate  whether to fire once immediately instead of waiting one period
    @return             `rospy.Timer`
    """
    period, mycallback = make_duration(period), lambda *_, **__: callback()
    if immediate:
        timer = rospy.Timer(rospy.Duration(nsecs=1), mycallback, oneshot=True)
    if not immediate or not oneshot:
        timer = rospy.Timer(period, mycallback, oneshot)
    return timer


def create_rate(frequency):
    """
    Returns a ROS1 rate instance, for sleeping at a fixed rate.

    @param   frequency  rate to sleep at, in Hz
    @return             `rospy.Rate`
    """
    return rospy.Rate(frequency)


def destroy_entity(item):
    """Closes the given publisher, subscriber, service client, or service server instance."""
    if isinstance(item, rospy.topics.Topic):
        item.unregister()
    elif isinstance(item, rospy.client.ServiceProxy):
        item.close()
    elif callable(getattr(item, "shutdown", None)):
        item.shutdown()  # Service or timer


def get_namespace():
    """Returns ROS1 node namespace."""
    return rospy.get_namespace()


def get_node_name():
    """Returns ROS1 node full name with namespace."""
    return rospy.get_name()


def get_nodes():
    """Returns all ROS1 nodes, as `[node full name, ]`."""
    # getSystemState() returns [[topic, [publishing node, ]], [..subs], [..services]]
    return sorted(set(l for s in MASTER.getSystemState()[-1] for _, ll in s for l in ll))


def get_topics():
    """Returns all available ROS1 topics, as `[(topic name, [type name, ]), ]`."""
    result = {}
    for n, t in MASTER.getTopicTypes()[-1]:
        result.setdefault(n, []).append(t)
    return sorted([n, sorted(map(canonical, tt))] for n, tt in result.items())


def get_services(node=None, namespace=None, include_types=True):
    """
    Returns all available ROS1 services, as `[(service name, [type name, ]), ]`.

    @param   node           full name of the node to return services for, if any
    @param   namespace      full or partial namespace to scope services from
    @param   include_types  if false, type names will be returned as an empty list
    """
    result = []
    for name in rosservice.get_service_list(node, namespace):
        if not include_types:
            result.append([name, []])
            continue  # for name
        try:
            typename = rosservice.get_service_type(name)
            if typename: result.append([name, [typename]])
        except Exception:
            logger.warning("Error retrieving service type for %s.", name, exc_info=True)
    return sorted(result)


def get_logger():
    """Returns `logging.Logger` for logging to ROS1 log handler."""
    return logger


def get_rostime():
    """Returns current ROS1 time, as `rospy.Time`."""
    result = rospy.get_rostime()
    if patch.PATCHED_FULL and not rospy.rostime.is_wallclock() \
    and rclify.clock.ClockType.ROS_TIME != result.clock_type:
        patch.set_extra_attribute(result, "_clock_type", rclify.clock.ClockType.ROS_TIME)
    return result


def remap_name(name, namespace=None):
    """
    Returns the absolute remapped topic/service name if mapping exists.

    @param   name       name to seek exact remapping for
    @param   namespace  namespace to resolve relative and private names to,
                        by default current node namespace
    @return             remapped resolved name, or original if not mapped
    """
    return rospy.remap_name(name, namespace)


def resolve_name(name, namespace=None):
    """
    Returns absolute remapped name, namespaced under current node if relative or private.

    @param   namespace  namespace to use if not current node full name
    """
    namespace = namespace or get_node_name()
    if not name:  # empty string resolves to node namespace
        return util.namesplit(namespace)[0] + "/"

    # Discard multiple slashes
    name2 = ("/" if name.startswith("/") else "") + "/".join(filter(bool, name.split("/")))
    if name2.startswith("~"):  # private name
        name2 = util.namejoin(namespace, name2.lstrip("~"))
    elif not name2.startswith("/"):  # relative name
        name2 = util.namejoin(util.namesplit(namespace)[0], name2)

    return rospy.names.get_resolved_mappings().get(name2, name2)


# -------------------------------- GENERAL API --------------------------------


def canonical(typename):
    """Returns "pkg/Type" for "pkg/subdir/Type"."""
    if typename.count("/") > 1:
        typename = "%s/%s" % tuple((x[0], x[-1]) for x in [typename.split("/")])[0]
    return typename


def format_param_name(name):
    """Returns parameter name using "/" separator, and leading root or private sigils stripped."""
    return name.replace(".", PARAM_SEPARATOR).lstrip("~" + PARAM_SEPARATOR)


def get_message_class(msg_or_type):
    """
    Returns ROS1 message / service class object.

    @param   msg_or_type  full or canonical class name
                          like "std_msgs/Bool" or "std_srvs/SetBool" or "std_srvs/SetBoolRequest",
                          or class instance like `std_msgs.msg.Bool()`,
                          or class object like `std_msgs.msg.Bool`
    @return               ROS1 message / service class object, like `std_msgs.msg.Bool`
                          or `std_srvs.srv.SetBool` or `std_srvs.srv.SetBoolRequest`,
                          or None if not found
    """
    if is_ros_message(msg_or_type) or is_ros_service(msg_or_type):
        return msg_or_type if inspect.isclass(msg_or_type) else type(msg_or_type)

    typename = canonical(msg_or_type)
    try: cls = roslib.message.get_message_class(typename)
    except Exception: cls = None
    if not cls:
        try: cls = roslib.message.get_service_class(typename)
        except Exception: cls = None
        if not cls and re.match(r".+_(Request|Response)$", typename):
            # Smooth over ROS1/ROS2 difference: ServiceRequest vs Service_Request
            typename = re.sub(r"(.+)_(Request|Response)$", r"\1\2", typename)
            try: cls = roslib.message.get_service_class(typename)
            except Exception: cls = None
    return cls


def get_message_definition(msg_or_type, full=True):
    """
    Returns ROS1 message or service request/response type definition text.

    Text will include subtype definitions by default.

    @param   msg_or_type  canonical or full class name like "std_msgs/Bool" or "std_msgs/msg/Bool",
                          or class instance like `std_msgs.msg.Bool()`,
                          or class object like `std_msgs.msg.Bool`
    @param   full         include definitions of nested types, separated by "\n---\nMSG: pkg/Type\n"
                          (ignored for service request/response types)
    @return               message type definition text
    """
    text = get_message_class(msg_or_type)._full_text
    if not full:  # Discard everything after a full line of "="
        text = re.sub(r"\n=+\n.+", "", text, flags=re.DOTALL)
    return text


def get_message_fields(val):
    """
    Returns {field name: field type name} if ROS1 message or service request/response, else {}.

    @param   val  ROS1 message or service request/response instance, or class object
    """
    names = getattr(val, "__slots__", [])
    if isinstance(val, tuple(ROS_TIME_CLASSES)):  # Empty __slots__
        names = genpy.TVal.__slots__
    return dict(zip(names, getattr(val, "_slot_types", [])))


def get_message_type(msg_or_cls):
    """
    Returns ROS1 message / service canonical type name, like "std_msgs/Header".

    @param   msg_or_cls  class instance like `std_msgs.msg.Bool()`,
                         or class object like `std_msgs.msg.Bool`
    """
    return msg_or_cls._type


def get_message_type_hash(msg_or_type):
    """
    Returns ROS1 message / service type MD5 hash.

    @param   msg_or_type  full or canonical class name
                          like "std_msgs/Bool" or "std_srvs/SetBool" or "std_srvs/SetBoolRequest",
                          or class instance like `std_msgs.msg.Bool()`,
                          or class object like `std_msgs.msg.Bool`
    """
    return get_message_class(msg_or_type)._md5sum


def get_message_value(msg, name):
    """Returns object attribute value, with numeric arrays converted to lists."""
    v = getattr(msg, name)
    do_list = isinstance(v, bytes) and get_message_fields(msg)[name].startswith("uint8[")
    return list(v) if do_list else v


def get_service_definition(srv_or_type):
    """
    Returns ROS1 service type definition text.

    @param   srv_or_type  canonical or full class name
                          like "std_srvs/SetBool" or "std_srvs/srv/SetBool",
                          or class instance like `std_srvs.srv.SetBool()`,
                          or class object like `std_srvs.srv.SetBool`
    @return               ROS service type definition text
    """
    cls = get_message_class(srv_or_type)
    reqcls, respcls = cls._request_class, cls._response_class
    reqtxt, resptxt = (get_message_definition(x, full=False) for x in (reqcls, respcls))
    return "%s\n---\n%s" % (reqtxt, resptxt)


def get_service_request_class(srv_or_type):
    """
    Returns ROS1 service request class object.

    @param   srv_or_type  canonical or full class name
                          like "std_srvs/SetBool" or "std_srvs/srv/SetBool",
                          or class instance like `std_srvs.srv.SetBool()`,
                          or class object like `std_srvs.srv.SetBool`
    @return               ROS1 service request class object, like `std_srvs.srv.SetBoolRequest`
    """
    return get_message_class(srv_or_type)._request_class


def get_service_response_class(srv_or_type):
    """
    Returns ROS1 service response class object.

    @param   srv_or_type  canonical or full class name
                          like "std_srvs/SetBool" or "std_srvs/srv/SetBool",
                          or class instance like `std_srvs.srv.SetBool()`,
                          or class object like `std_srvs.srv.SetBool`
    @return               ROS1 service response class object, like `std_srvs.srv.SetBoolResponse`
    """
    return get_message_class(srv_or_type)._response_class


def is_ros_message(val):
    """
    Returns whether value is a ROS1 message or service request/response class or instance.

    @param   val  like `std_msgs.msg.Bool()` or `std_srvs.srv.SetBoolRequest`
    @return       True if value is a ROS1 message or service request/response class or instance,
                  False otherwise
    """
    return (issubclass if inspect.isclass(val) else isinstance)(val, (genpy.Message, genpy.TVal))


def is_ros_service(val):
    """Returns whether value is a ROS1 service class object or instance."""
    STRFIELDS, MSGFIELDS = ('_md5sum', '_type'), ('_request_class', '_response_class')
    return all(isinstance(getattr(val, x, None), str) for x in STRFIELDS) and \
           all(is_ros_message(getattr(val, x, None)) for x in MSGFIELDS)


def is_ros_time(val):
    """Returns whether value is a ROS1 time/duration."""
    return isinstance(val, genpy.TVal)


def make_duration(secs=0, nsecs=0):
    """Returns a ROS1 duration, as rospy.Duration."""
    return rospy.Duration(secs=secs, nsecs=nsecs)


def make_time(secs=0, nsecs=0):
    """Returns a ROS1 time, as rospy.Time."""
    return rospy.Time(secs=secs, nsecs=nsecs)


def serialize_message(msg):
    """Returns ROS1 message or service request/response as a serialized binary of `bytes()`."""
    buf = io.BytesIO()
    msg.serialize(buf)
    return buf.getvalue()


def deserialize_message(raw, cls_or_typename):
    """Returns ROS1 message or service request/response instantiated from serialized binary."""
    return get_message_class(cls_or_typename)().deserialize(raw)


def scalar(typename):
    """
    Returns scalar type from ROS message data type, like "uint8" from "uint8[100]".

    Returns type unchanged if already a scalar.
    """
    return typename[:typename.index("[")] if "[" in typename else typename


def to_nsec(val):
    """Returns value in nanoseconds if value is ROS1 time/duration, else value."""
    return val.to_nsec() if isinstance(val, genpy.TVal) else val


def to_sec(val):
    """Returns value in seconds if value is ROS1 time/duration, else value."""
    return val.to_sec() if isinstance(val, genpy.TVal) else val


def to_sec_nsec(val):
    """Returns value as (seconds, nanoseconds) if value is ROS1 time/duration, else value."""
    return (val.secs, val.nsecs) if isinstance(val, genpy.TVal) else val


__all__ = [
    "ROSLogHandler", "PARAM_SEPARATOR", "PRIVATE_PREFIX", "ROS_ALIAS_TYPES",
    "ROS_TIME_CLASSES", "ROS_TIME_TYPES",
    "canonical", "create_client", "create_publisher", "create_rate", "create_service",
    "create_subscriber", "create_timer", "delete_param", "deserialize_message",
    "destroy_entity", "format_param_name", "get_logger", "get_message_class",
    "get_message_definition", "get_message_fields", "get_message_type",
    "get_message_type_hash", "get_message_value", "get_namespace", "get_node_name",
    "get_nodes", "get_param", "get_param_names", "get_params", "get_rostime",
    "get_service_definition", "get_service_request_class", "get_service_response_class",
    "get_services", "get_topics", "has_param", "init_node", "init_params", "is_ros_message",
    "is_ros_service", "is_ros_time", "make_duration", "make_time", "ok", "register_init",
    "remap_name", "resolve_name", "scalar", "serialize_message", "set_param", "shutdown",
    "spin", "spin_once", "spin_until_future_complete", "start_spin", "to_nsec", "to_sec",
    "to_sec_nsec"
]
