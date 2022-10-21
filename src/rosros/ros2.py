# -*- coding: utf-8 -*-
"""
ROS2 interface.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     11.02.2022
@modified    21.10.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.ros2
import array
import inspect
import logging
import os
import re
import threading
import time
import traceback

import ament_index_python
import builtin_interfaces.msg
import rclpy
import rclpy.callback_groups
import rclpy.client
import rclpy.clock
import rclpy.duration
import rclpy.exceptions
import rclpy.executors
import rclpy.logging
import rclpy.publisher
import rclpy.qos
import rclpy.serialization
import rclpy.service
import rclpy.subscription
import rclpy.time
import rclpy.timer
import rosidl_runtime_py.utilities
from rclpy.impl.implementation_singleton import rclpy_implementation as rclpy_extension

from . import parsing
from . import patch
from . import util


## Stand-in for `rospy.AnyMsg` with equivalent interface
AnyMsg = patch.AnyMsg

## ROS2 time/duration message types
ROS_TIME_TYPES = ["builtin_interfaces/Time", "builtin_interfaces/Duration"]

## ROS2 time/duration types and message types mapped to type names
ROS_TIME_CLASSES = {rclpy.time.Time:                 "builtin_interfaces/Time",
                    builtin_interfaces.msg.Time:     "builtin_interfaces/Time",
                    rclpy.duration.Duration:         "builtin_interfaces/Duration",
                    builtin_interfaces.msg.Duration: "builtin_interfaces/Duration"}

## ROS2 time/duration types mapped to message types
ROS_TIME_MESSAGES = {rclpy.time.Time:          builtin_interfaces.msg.Time,
                     rclpy.duration.Duration:  builtin_interfaces.msg.Duration}

## Mapping between ROS type aliases and real types, like {"byte": "uint8"}
ROS_ALIAS_TYPES = {"byte": "uint8", "char": "int8"}

## Data Distribution Service types to ROS builtins
DDS_TYPES = {"boolean":             "bool",
             "float":               "float32",
             "double":              "float64",
             "octet":               "byte",
             "short":               "int16",
             "unsigned short":      "uint16",
             "long":                "int32",
             "unsigned long":       "uint32",
             "long long":           "int64",
             "unsigned long long":  "uint64", }

## Separator char between ROS2 parameter namespace parts
PARAM_SEPARATOR = "."

## Prefix for "private" names, auto-namespaced under current namespace
PRIVATE_PREFIX = "~"

## ROS Python module family, "rclpy"
FAMILY = "rclpy"

## `rclpy.node.Node` instance
NODE = None

## `rclpy.callback_groups.CallbackGroup` instance
CALLBACK_GROUP = None

## `rclpy.executors.Executor` instance
EXECUTOR = None

## `threading.Thread` running `EXECUTOR.spin()`
SPINNER = None

## Flag for `shutdown()` been called
SHUTDOWN = False


logger = util.ThrottledLogger(logging.getLogger())


class ROSLogHandler(logging.Handler):
    """Logging handler that forwards logging messages to ROS2 logger."""

    def __init__(self, node):
        super().__init__()
        self._logger = node.get_logger()  # rclpy.impl.rcutils_logger.RcutilsLogger instance


    def emit(self, record):
        """Adds message to ROS2 logger."""
        if not self._logger.is_enabled_for(record.levelno):
            return
        try:
            text = record.msg % record.args if record.args else record.msg
        except Exception:
            text = record.msg
        if record.exc_info:
            text += "\n\n" + "".join(traceback.format_exception(*record.exc_info))
        # Cannot use self._logger.log(text, level)
        # as rclpy raises error if using different levels from same line
        if   logging.DEBUG == record.levelno:
            self._logger.debug(text)
        elif logging.INFO  == record.levelno:
            self._logger.info(text)
        elif logging.WARN  == record.levelno:
            self._logger.warning(text)
        elif logging.ERROR == record.levelno:
            self._logger.error(text)
        elif logging.FATAL == record.levelno:
            self._logger.fatal(text)
        else:
            self._logger.info(text)


class Mutex:
    """Container for local mutexes."""

    ## Mutex for `init_node()` and `shutdown()`
    NODE       = threading.Lock()

    ## Mutex for `spin()`
    SPIN       = threading.RLock()

    ## Mutex for `spin_once()` and `spin_until_future_complete()`
    SPIN_ONCE  = threading.RLock()

    ## Mutex for `start_spin()`
    SPIN_START = threading.RLock()



def init_node(name, args=None, namespace=None, anonymous=False, log_level=None, enable_rosout=True,
              multithreaded=True, reentrant=False):
    """
    Initializes rclpy and creates ROS2 node.

    @param   name           node name, without namespace
    @param   args           list of command-line arguments for the node
    @param   namespace      node namespace override
    @param   anonymous      whether to auto-generate a unique name for the node,
                            using the given name as base
    @param   log_level      level to set for ROS logging
                            (name like "DEBUG" or one of `logging` constants like `logging.DEBUG`)
    @param   enable_rosout  `False` to suppress auto-publication of rosout
    @param   multithreaded  use `MultiThreadedExecutor` instead of `SingleThreadedExecutor`
    @param   reentrant      use `ReentrantCallbackGroup` instead of `MutuallyExclusiveCallbackGroup`
    """
    global NODE, CALLBACK_GROUP, EXECUTOR, SHUTDOWN
    if NODE: return

    if anonymous:
        name = "%s_%s_%s" % (name, os.getpid(), int(time.time_ns() * 1000))
    with Mutex.NODE:
        if NODE: return

        patch.patch_ros2()
        logger.debug("Initializing ROS node %r.", name)
        try: rclpy.init(args=args)
        except RuntimeError: pass  # Raises if called twice at runtime
        NODE = rclpy.create_node(name, namespace=namespace, enable_rosout=enable_rosout,
                                 automatically_declare_parameters_from_overrides=True,
                                 allow_undeclared_parameters=True)
        execls = rclpy.executors.SingleThreadedExecutor
        grpcls = rclpy.callback_groups.MutuallyExclusiveCallbackGroup
        if multithreaded: execls = rclpy.executors.MultiThreadedExecutor
        if reentrant:     grpcls = rclpy.callback_groups.ReentrantCallbackGroup
        EXECUTOR = execls()
        EXECUTOR.add_node(NODE)
        defgrp = NODE.default_callback_group
        CALLBACK_GROUP = defgrp if grpcls is type(defgrp) else grpcls()
        SHUTDOWN = False

        if not any(isinstance(x, ROSLogHandler) for x in logger.handlers):
            logger.addHandler(ROSLogHandler(NODE))
        if log_level is not None:
            if not isinstance(log_level, str): log_level = logging.getLevelName(log_level)
            logger.setLevel(log_level)
            ros_level = rclpy.logging.get_logging_severity_from_string(log_level)
            NODE.get_logger().set_level(ros_level)


def register_init(node):
    """
    Informs `rosros` of ROS2 having been initialized outside of `init_node()`.

    @param   node  `rclpy.node.Node` instance to use for ROS operations
    """
    if not isinstance(node, rclpy.node.Node):
        raise TypeError("register_init() argument must be rclpy Node, not %r" %
                        node.__class__.__name__)
    global NODE, EXECUTOR
    NODE, EXECUTOR = node, node.executor or rclpy.get_global_executor()
    patch.patch_ros2()
    if not any(isinstance(x, ROSLogHandler) for x in logger.handlers):
        logger.addHandler(ROSLogHandler(NODE))


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
        elif not NODE.has_parameter(k):
            NODE.declare_parameter(format_param_name(k), v)

    for path, param in NODE.get_parameters_by_prefix("").items():
        if param.value is not None:
            util.set_value(result, [x for x in path.split(PARAM_SEPARATOR) if x], param.value)
    logger.debug("Initialized node parameters %r.", result)
    return result


def has_param(name):
    """
    Returns whether the parameter exists.

    @param   name  full name of the parameter in node namespace
    """
    return NODE.has_parameter(format_param_name(name))


def get_param(name, default=None, autoset=True):
    """
    Returns parameter value from current ROS1 node.

    @param   default     optional default to return if parameter unknown
    @param   autoset     set default value to node parameter if unknown
    @return              parameter value, or default if parameter was unknown

    @throws  KeyError    if parameter not set and default not given
    """
    name = format_param_name(name)
    if not NODE.has_parameter(name):
        if not autoset or default is None:
            raise KeyError("Parameter %r is not set" % name)
        NODE.declare_parameter(name, default)
    return NODE.get_parameter(name).value


def get_param_names():
    """Returns the names of all current ROS2 node parameters."""
    return list(NODE.get_parameters_by_prefix(""))


def get_params(nested=True):
    """
    Returns the current ROS2 node parameters, by default as nested dictionary.

    @param   nested  return a nested dictionary,
                     like `{"my": {"name": 1}}` vs {"my.name": 1}
    """
    result = {}
    for path, param in NODE.get_parameters_by_prefix("").items():
        key = [x for x in path.split(PARAM_SEPARATOR) if x] if nested else path
        util.set_value(result, key, param.value)
    return result


def set_param(name, value, descriptor=None):
    """
    Sets a parameter on the node, auto-declaring it if unknown so far.

    @param    name        parameter full name
    @param    value       parameter value to set
    @param    descriptor  optional `rcl_interfaces.msg.ParameterDescriptor`
    @return               the set value
    """
    name = format_param_name(name)
    if not NODE.has_parameter(name):
        NODE.declare_parameter(name, descriptor=descriptor)
    result = NODE.set_parameters([rclpy.Parameter(name, value=value)])[0]
    if not result.successful:
        raise Exception(result.reason)
    return value


def delete_param(name):
    """
    Deletes parameter from the node.

    @param   name      full name of the parameter in node namespace
    @throws  KeyError  if parameter not set
    """
    name = format_param_name(name)
    try:
        NODE.undeclare_parameter(name)
    except rclpy.exceptions.ParameterNotDeclaredException:
        raise KeyError(name)


def ok():
    """Returns whether ROS2 has been initialized and is not shut down."""
    return rclpy.ok()


def start_spin():
    """Sets ROS2 node spinning forever in a background thread."""
    def do_spin():
        global SPINNER
        try:
            while SPINNER and EXECUTOR and ok(): EXECUTOR.spin_once(0.5)
        finally: SPINNER = None

    global SPINNER
    with Mutex.SPIN_START:
        if SPINNER or Mutex.SPIN._is_owned(): return
        SPINNER = threading.Thread(target=do_spin)
        SPINNER.start()


def spin():
    """Spins ROS2 node forever."""
    global SPINNER

    if SPINNER:
        try: SPINNER.join()  # Wait on background thread instead
        except KeyboardInterrupt: SPINNER = None  # Signal thread to end
        return

    if Mutex.SPIN._is_owned():  # spin() started from another thread
        with Mutex.SPIN: return  # Wait on mutex instead
    with Mutex.SPIN:
        EXECUTOR.spin()


def spin_once(timeout=None):
    """
    Executes one pending ROS2 operation or waits until timeout.

    Waits forever if timeout not given or negative.

    @param  timeout  time to wait at most, as seconds or ROS2 duration;
                     None or <0 waits forever
    """
    global SPINNER
    timeout = to_sec(timeout)
    if SPINNER or Mutex.SPIN_ONCE._is_owned():  # Executor already spinning: sleep instead
        timeout = 2**31 - 1 if timeout is None or timeout < 0 else timeout
        group = rclpy.callback_groups.ReentrantCallbackGroup()
        timer = NODE.create_timer(timeout, callback=None, callback_group=group)
        rclpy.timer.Rate(timer, context=NODE.context).sleep()
        NODE.destroy_timer(timer)
        return
    with Mutex.SPIN_ONCE:
        EXECUTOR.spin_once(timeout_sec=timeout)


def spin_until_future_complete(future, timeout=None):
    """
    Spins ROS2 until future complete or timeout reached or ROS shut down.

    @param  future   object with `asyncio.Future`-conforming interface to complete
                     (will not be awaited with `async`)
    @param  timeout  time to wait, as seconds or ROS2 duration
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
    """Shuts down ROS2 execution, if any."""
    global NODE, EXECUTOR, SPINNER, SHUTDOWN
    if SHUTDOWN: return

    with Mutex.NODE:
        if SHUTDOWN: return

        node, executor = NODE, EXECUTOR
        rclpy.ok() and rclpy.shutdown()
        NODE, EXECUTOR, SPINNER, SHUTDOWN = None, None, None, True
        executor and executor.shutdown()
        node and node.destroy_node()


def create_client(service, cls_or_typename, **qosargs):
    """
    Returns a ROS2 service client instance, for invoking a service.

    @param   service          name of service to invoke
    @param   cls_or_typename  ROS2 service class object like `std_srvs.srv.SetBool`
                              or service type name like `"std_srvs/SetBool"
    @param   qosargs          additional key-value arguments for ROS2
                              `QoSProfile`, like `reliability`
    @return                   `rclpy.client.Client`, will support keyword arguments in calls
                              and be callable itself
    """
    cls = get_message_class(cls_or_typename)
    if qosargs: qos = rclpy.qos.QoSProfile(**qosargs)
    else: qos = rclpy.qos.QoSPresetProfiles.SERVICES_DEFAULT.value
    return NODE.create_client(cls, service, qos_profile=qos,
                              callback_group=CALLBACK_GROUP)


def create_service(service, cls_or_typename, callback, **qosargs):
    """
    Returns a ROS2 service server instance, for providing a service.

    @param   service          name of service to provide
    @param   cls_or_typename  ROS2 service class object like `std_srvs.srv.SetBool`
                              or service type name like `"std_srvs/SetBool"
    @param   callback         callback function, invoked with `(request, response)`,
                              expected to return the populated response,
                              or a list/tuple/dictionary for populating the response.
                              If the function only takes one argument,
                              it is invoked with `(request)`.
    @param   qosargs          additional key-value arguments for ROS2
                              `QoSProfile`, like `reliability`
    @return                   `rclpy.service.Service`
    """
    cls = get_message_class(cls_or_typename)
    if qosargs: qos = rclpy.qos.QoSProfile(**qosargs)
    else: qos = rclpy.qos.QoSPresetProfiles.SERVICES_DEFAULT.value
    return NODE.create_service(cls, service, util.wrap_arity(callback),
                               qos_profile=qos, callback_group=CALLBACK_GROUP)


def create_publisher(topic, cls_or_typename, latch=False, queue_size=0, **qosargs):
    """
    Returns a ROS2 publisher instance.

    @param   topic            name of topic to open
    @param   cls_or_typename  ROS2 message class object like `std_msgs.msg.Bool`
                              or message type name like "std_msgs/Bool"
    @param   latch            provide last published message to new subscribers
                              (sets `DurabilityPolicy.TRANSIENT_LOCAL`)
    @param   queue_size       queue size of outgoing messages (0 or None: infinite)
    @param   qosargs          additional key-value arguments for ROS2
                              `QoSProfile`, like `reliability`
    @return                   `rclpy.publisher.Publisher`,
                              will support keyword arguments in `publish()`
                              and have `get_num_connections()`
    """
    cls = get_message_class(cls_or_typename)
    qosargs.setdefault("depth", queue_size or 0)
    if latch: qosargs["durability"] = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    qos = rclpy.qos.QoSProfile(**qosargs)
    return NODE.create_publisher(cls, topic, qos, callback_group=CALLBACK_GROUP)


def create_subscriber(topic, cls_or_typename, callback, callback_args=None,
                      queue_size=0, raw=False, **qosargs):
    """
    Returns a ROS2 subscriber instance.

    @param   topic            name of topic to listen to
    @param   cls_or_typename  ROS2 message class object like `std_msgs.msg.Bool`
                              or message type name like "std_msgs/Bool"
    @param   callback         callback function, invoked with received message,
                              and with additional arguments if given
    @param   callback_args    additional arguments to pass to the callback, if any,
                              invoked as `callback(msg, callback_args)´
    @param   queue_size       queue size of incoming messages (0 or None: infinite)
    @param   raw              invoke callback with serialized message bytes
    @param   qosargs          additional key-value arguments for ROS2
                              `QoSProfile`, like `reliability`.
                              `__autodetect` will look up current publishers on the topic
                              and create a compatible QoS.
    @return                   `rclpy.subscription.Subscription`
    """
    qosargs.setdefault("depth", queue_size or 0)
    if qosargs.pop("__autodetect", False):
        qosargs.update(get_topic_qos(topic, cls_or_typename, queue_size or 0))
    cls = get_message_class(cls_or_typename)
    qos = rclpy.qos.QoSProfile(**qosargs)
    sub = NODE.create_subscription(cls, topic, callback, qos, raw=raw,
                                   callback_group=CALLBACK_GROUP)
    if callback_args is not None:
        sub._callbacks = [(callback, callback_args)]
    return sub


def create_timer(period, callback, oneshot=False, immediate=False):
    """
    Returns a ROS2 timer instance.

    @param   period     desired period between callbacks, as seconds or ROS duration
    @param   callback   callback function invoked on timer, with no arguments
    @param   oneshot    whether to fire only once
    @param   immediate  whether to fire once immediately instead of waiting one period
    @return             `rclpy.timer.Timer`
    """
    oneshot_timer = None
    def oneshot_callback():
        oneshot_timer.destroy()
        callback and callback()

    period, kws = to_sec(period), dict(callback_group=CALLBACK_GROUP)
    if immediate:
        timer = oneshot_timer = NODE.create_timer(1e-9, oneshot_callback, **kws)
    if not immediate or not oneshot:
        if oneshot:
            timer = oneshot_timer = NODE.create_timer(period, oneshot_callback, **kws)
        else:
            timer = NODE.create_timer(period, callback, **kws)
    return timer


def create_rate(frequency):
    """
    Returns a ROS2 rate instance, for sleeping at a fixed rate.

    @param   frequency  rate to sleep at, in Hz
    @return             `rclpy.timer.Rate`
    """
    return NODE.create_rate(frequency)


def destroy_entity(item):
    """Closes the given publisher, subscriber, service client, service server, timer, or rate instance."""
    try:
        if   isinstance(item, rclpy.client.Client):             NODE.destroy_client(item)
        elif isinstance(item, rclpy.service.Service):           NODE.destroy_service(item)
        elif isinstance(item, rclpy.publisher.Publisher):       NODE.destroy_publisher(item)
        elif isinstance(item, rclpy.timer.Rate):                NODE.destroy_rate(item)
        elif isinstance(item, rclpy.subscription.Subscription): NODE.destroy_subscription(item)
        elif isinstance(item, rclpy.timer.Timer):               NODE.destroy_timer(item)
        else:
            item = None
    finally:
        try: item and item.destroy()
        except Exception: pass


def get_namespace():
    """Returns ROS2 node namespace."""
    return NODE.get_namespace()


def get_node_name():
    """Returns ROS2 node full name with namespace."""
    return util.namejoin(NODE.get_namespace(), NODE.get_name())


def get_nodes():
    """Returns all ROS2 nodes, as `[node full name, ]`."""
    return [util.namejoin(b, a) for a, b in NODE.get_node_names_and_namespaces()]


def get_topics():
    """Returns all available ROS2 topics, as `[(topic name, [type name, ]), ]`."""
    return sorted([n, sorted(map(canonical, tt))] for n, tt in NODE.get_topic_names_and_types())


def get_topic_qos(topic, cls_or_typename, queue_size=10, publish=False):
    """
    Returns dictionary for populating rclpy.qos.QoSProfile compatible with counterparts on topic.

    @param   topic            topic full name
    @param   cls_or_typename  message type class or name
    @param   queue_size       populates "depth"
    @param   publish          whether to return QoS settings for creating a publisher,
                              by default returns settings for a subscription
    @return                   {"depth", "reliability", "durability"}
    """
    typename = cls_or_typename
    if not isinstance(typename, str): typename = get_message_type(cls_or_typename)
    args = dict(depth=queue_size, reliability=rclpy.qos.ReliabilityPolicy.SYSTEM_DEFAULT,
                durability=rclpy.qos.DurabilityPolicy.SYSTEM_DEFAULT)
    qry = NODE.get_subscriptions_info_by_topic if publish else NODE.get_publishers_info_by_topic
    qoses = [x.qos_profile for x in qry(topic) if canonical(x.topic_type) == typename]
    rels, durs = zip(*[(x.reliability, x.durability) for x in qoses]) if qoses else ([], [])
    # If subscription demands stricter QoS than publisher offers, no messages are received
    if rels: args.update(reliability=max(rels))  # DEFAULT < RELIABLE < BEST_EFFORT
    if durs: args.update(durability =max(durs))  # DEFAULT < TRANSIENT_LOCAL < VOLATILE
    return args


def get_services(node=None, namespace=None, include_types=True):
    """
    Returns all available ROS2 services, as `[(service name, [type name, ]), ]`.

    @param   node           full name of the node to return services for, if any
    @param   namespace      full or partial namespace to scope services from
    @param   include_types  if false, type names will be returned as an empty list
    """
    services = NODE.get_service_names_and_types() if not node else \
               NODE.get_service_names_and_types_by_node(**util.namesplit(node)[::-1])
    return [[n, sorted(map(canonical, tt)) if include_types else []]
            for n, tt in sorted(services) if not namespace or n.startswith(namespace)]


def get_logger():
    """
    Returns `logging.Logger` for logging to ROS2 log handler.

    Logging methods on the logger (`debug()`, `info()`, etc) accept additional keyword arguments:
    - `__once__`:                whether to log only once from call site
    - `__throttle__`:            seconds to skip logging from call site for
    - `__throttle_identical__`:  whether to skip logging identical consecutive texts from call site
                                 (given log message excluding formatting arguments).
                                 Combines with `__throttle__` to skip duplicates for a period.
    """
    return logger


def get_rostime():
    """Returns current ROS2 time, as `rclpy.time.Time`."""
    return NODE.get_clock().now()


def remap_name(name, namespace=None):
    """
    Returns the absolute remapped topic/service name if mapping exists.

    @param   name       name to seek exact remapping for
    @param   namespace  namespace to resolve relative and private names to,
                        by default current node namespace
    @return             remapped resolved name, or original if not mapped
    """
    name1 = _resolve_name(name, namespace)
    with NODE.handle as node_capsule:
        name2 = rclpy_extension.rclpy_remap_topic_name(node_capsule, name1)
    return name2 if (name1 != name2) else name


def resolve_name(name, namespace=None):
    """
    Returns absolute remapped name, namespaced under current node if relative or private.

    @param   namespace  namespace to use if not current node full name
    """
    name1 = _resolve_name(name, namespace)
    with NODE.handle as node_capsule:
        return rclpy_extension.rclpy_remap_topic_name(node_capsule, name1)


def _resolve_name(name, namespace=None):
    """
    Returns absolute name, namespaced under current node if relative or private.

    @param   namespace  namespace to use if not current node full name
    """
    namespace = namespace or get_node_name()
    if not name:  # empty name resolves to node namespace
        return util.namesplit(namespace)[0] + "/"

    # Discard multiple slashes
    name2 = ("/" if name.startswith("/") else "") + "/".join(filter(bool, name.split("/")))
    if name2.startswith(PRIVATE_PREFIX):  # private name
        name2 = util.namejoin(namespace, name2.lstrip(PRIVATE_PREFIX))
    elif not name2.startswith("/"):  # relative name
        name2 = util.namejoin(util.namesplit(namespace)[0], name2)
    return name2


# -------------------------------- GENERAL API --------------------------------


def canonical(typename):
    """
    Returns "pkg/Type" for "pkg/msg/Type", standardizes various ROS2 formats.

    Converts DDS types like "octet" to "byte", and "sequence<uint8, 100>" to "uint8[100]".
    """
    is_array, bound, dimension = False, "", ""

    match = re.match("sequence<(.+)>", typename)
    if match:  # "sequence<uint8, 100>" or "sequence<uint8>"
        is_array = True
        typename = match.group(1)
        match = re.match(r"([^,]+)?,\s?(\d+)", typename)
        if match:  # sequence<uint8, 10>
            typename = match.group(1)
            if match.lastindex > 1: dimension = match.group(2)

    match = re.match("(w?string)<(.+)>", typename)
    if match:  # string<5>
        typename, bound = match.groups()

    if "[" in typename:  # "string<=5[<=10]" or "string<=5[10]"
        dimension = typename[typename.index("[") + 1:typename.index("]")]
        typename, is_array = typename[:typename.index("[")], True

    if "<=" in typename:  # "string<=5"
        typename, bound = typename.split("<=")

    if typename.count("/") > 1:
        typename = "%s/%s" % tuple((x[0], x[-1]) for x in [typename.split("/")])[0]

    suffix = ("<=%s" % bound if bound else "") + ("[%s]" % dimension if is_array else "")
    return DDS_TYPES.get(typename, typename) + suffix


def format_param_name(name):
    """Returns parameter name using "." separator, and leading root or private sigils stripped."""
    return name.replace("/", PARAM_SEPARATOR).lstrip(PRIVATE_PREFIX + PARAM_SEPARATOR)


def get_message_class(msg_or_type):
    """
    Returns ROS2 message / service class object.

    @param   msg_or_type  full or canonical class name,
                          like "std_msgs/Bool" or "std_srvs/SetBool" or "std_srvs/SetBoolRequest";
                          or class instance like `std_msgs.msg.Bool()`
    @return               ROS2 message / service class object, like `std_msgs.msg.Bool`
                          or `std_srvs.srv.SetBool` or `std_srvs.srv.SetBoolRequest`,
                          or None if not found
    """
    if is_ros_message(msg_or_type) or is_ros_service(msg_or_type):
        return msg_or_type if inspect.isclass(msg_or_type) else type(msg_or_type)

    cls = None
    try:
        cls = rosidl_runtime_py.utilities.get_message(msg_or_type)
    except Exception:
        part = "Request"  if re.match(r".+\wRequest$",  msg_or_type) else \
               "Response" if re.match(r".+\wResponse$", msg_or_type) else None
        if part:
            msg_or_type = msg_or_type[:-len(part)]
            # Smooth over ROS1/ROS2 difference: ServiceRequest vs Service_Request
            if msg_or_type.endswith("_"): msg_or_type = msg_or_type[:-1]
        try: cls = rosidl_runtime_py.utilities.get_service(msg_or_type)
        except Exception: pass
        if cls and part:
            cls = getattr(cls, part)
    return cls


def get_message_definition(msg_or_type, full=True):
    """
    Returns ROS2 message or service request/response type definition text.

    Text will include subtype definitions by default for messages.

    @param   msg_or_type  canonical or full class name like "std_msgs/Bool" or "std_msgs/msg/Bool",
                          or class instance like `std_msgs.msg.Bool()`,
                          or class object like `std_msgs.msg.Bool`
    @param   full         include definitions of nested types, separated by "\n---\nMSG: pkg/Type\n"
                          (ignored for service request/response types)
    @return               message type definition text
    """
    typename = canonical(msg_or_type) if isinstance(msg_or_type, str) else \
               get_message_type(msg_or_type)
    try: result = parsing.get_ros2_message_definition(typename, full)
    except Exception: result = None
    if result is None:  # Assume service request/response
        part = "Request"  if re.match(r".+\wRequest$",  typename) else \
               "Response" if re.match(r".+\wResponse$", typename) else None
        # Smooth over ROS1/ROS2 difference: ServiceRequest vs Service_Request
        typename = re.sub(r"(.+)_(Request|Response)$", r"\1", typename)
        typename = re.sub(r"(.+)(Request|Response)$",  r"\1", typename)
        result = parsing.get_ros2_service_definition(typename)
        if part:
            result = result.split("---\n", 1)["Response" == part]
    if result is None:
        raise LookupError("Could not find the interface %r" % msg_or_type)
    return result


def get_message_fields(val):
    """
    Returns {field name: field type name} if ROS2 message or service request/response, else {}.

    @param   val  ROS2 message or service request/response instance, or class object
    """
    if not is_ros_message(val): return {}
    return {k: canonical(v) for k, v in val.get_fields_and_field_types().items()}


def get_message_header(val):
    """
    Returns message `std_msgs/Header`-attribute if any, else `None`.

    @param   val  ROS message or service request/response instance
    """
    if is_ros_message(val):
        for name, typename in val.get_fields_and_field_types().items():
            if "std_msgs/Header" == typename:
                return get_message_value(val, name)
    return None


def get_message_type(msg_or_cls):
    """
    Returns ROS2 message / service canonical type name, like "std_msgs/Header".

    Returns "*" for `AnyMsg`.

    @param   msg_or_cls  class instance like `std_msgs.msg.Bool()`,
                         or class object like `std_msgs.msg.Bool`
    @return              canonical name, or `None` if not ROS message / service
    """
    cls = msg_or_cls if inspect.isclass(msg_or_cls) else type(msg_or_cls)
    if not is_ros_message(cls) and not is_ros_time(cls) and not is_ros_service(cls):
        return None
    return canonical("%s/%s" % (cls.__module__.split(".")[0], cls.__name__))


def get_message_type_hash(msg_or_type):
    """
    Returns ROS2 message / service type MD5 hash.

    @param   msg_or_type  full or canonical class name
                          like "std_msgs/Bool" or "std_srvs/SetBool" or "std_srvs/SetBoolRequest",
                          or class instance like `std_msgs.msg.Bool()`,
                          or class object like `std_msgs.msg.Bool`
    """
    typename = msg_or_type if isinstance(msg_or_type, str) else get_message_type(msg_or_type)
    return _get_message_type_hash(canonical(typename))


@util.memoize
def _get_message_type_hash(typename):
    """Returns ROS2 message type MD5 hash (internal caching method)."""
    definition = get_message_definition(typename)
    return parsing.calculate_definition_hash(typename, definition)


def get_message_value(msg, name):
    """
    Returns object attribute value, with numeric arrays converted to lists.

    @param   message attribute name; may also be (nested, path) or "nested.path"
    """
    v, parent, k = util.get_nested(msg, name)
    if isinstance(v, (bytes, array.array)) \
    or "numpy.ndarray" == "%s.%s" % (v.__class__.__module__, v.__class__.__name__):
        v = list(v)
    if v and isinstance(v, (list, tuple)) and is_ros_message(parent):
        typename = canonical(parent.get_fields_and_field_types()[k])
        scalartype = scalar(typename)
        if scalartype in ("byte", "uint8"):
            if isinstance(v[0], bytes):
                v = list(map(ord, v))  # In ROS2, a byte array like [0, 1] is [b"\0", b"\1"]
            elif scalartype == typename:
                v = v[0]  # In ROS2, single byte values are given as bytes()
    return v


def get_package_share_directory(name):
    """
    Returns the share directory of the package.

    For example, "/opt/ros/foxy/share/name" or "/home/ros1_ws/install/share/name".

    @throws  KeyError  if package not found
    """
    try:
        return ament_index_python.get_package_share_directory(name)
    except ament_index_python.PackageNotFoundError as e:
        raise KeyError(name) from e


def get_service_definition(srv_or_type):
    """
    Returns ROS2 service type definition text.

    @param   srv_or_type  canonical or full class name
                          like "std_srvs/SetBool" or "std_srvs/srv/SetBool",
                          or class instance like `std_srvs.srv.SetBool()`,
                          or class object like `std_srvs.srv.SetBool`
    @return               ROS2 service type definition text
    """
    typename = srv_or_type if isinstance(srv_or_type, str) else get_message_type(srv_or_type)
    return parsing.get_ros2_service_definition(canonical(typename))


def get_service_request_class(srv_or_type):
    """
    Returns ROS2 service request class object.

    @param   srv_or_type  canonical or full class name
                          like "std_srvs/SetBool" or "std_srvs/srv/SetBool",
                          or class instance like `std_srvs.srv.SetBool()`,
                          or class object like `std_srvs.srv.SetBool`
    @return               ROS2 service request class object, like `std_srvs.srv.SetBool.Request`
    """
    return get_message_class(srv_or_type).Request


def get_service_response_class(srv_or_type):
    """
    Returns ROS2 service response class object.

    @param   srv_or_type  canonical or full class name
                          like "std_srvs/SetBool" or "std_srvs/srv/SetBool",
                          or class instance like `std_srvs.srv.SetBool()`,
                          or class object like `std_srvs.srv.SetBool`
    @return               ROS2 service response class object, like `std_srvs.srv.SetBool.Response`
    """
    return get_message_class(srv_or_type).Response


def is_ros_message(val):
    """
    Returns whether value is a ROS1 message or service request/response class or instance.

    @param   val  like `std_msgs.msg.Bool()` or `std_srvs.srv.SetBoolRequest`
    @return       True if value is a ROS1 message or service request/response class or instance,
                  False otherwise
    """
    return isinstance(val, AnyMsg) or issubclass(val, AnyMsg) or \
           rosidl_runtime_py.utilities.is_message(val)


def is_ros_service(val):
    """Returns whether value is a ROS2 service class object."""
    return rosidl_runtime_py.utilities.is_service(val)


def is_ros_time(val):
    """Returns whether value is a ROS2 time/duration class or instance."""
    if inspect.isclass(val): return issubclass(val, tuple(ROS_TIME_CLASSES))
    return isinstance(val, tuple(ROS_TIME_CLASSES))


def make_duration(secs=0, nsecs=0):
    """Returns an rclpy.duration.Duration."""
    return rclpy.duration.Duration(seconds=secs, nanoseconds=nsecs)


def make_time(secs=0, nsecs=0):
    """Returns a ROS2 time, as rclpy.time.Time."""
    return rclpy.time.Time(seconds=secs, nanoseconds=nsecs)


def serialize_message(msg):
    """Returns ROS2 message or service request/response as a serialized binary of `bytes()`."""
    return rclpy.serialization.serialize_message(msg)


def deserialize_message(raw, cls_or_typename):
    """Returns ROS2 message or service request/response instantiated from serialized binary."""
    return rclpy.serialization.deserialize_message(raw, get_message_class(cls_or_typename))


def scalar(typename):
    """
    Returns unbounded scalar type from ROS2 message data type

    Like "uint8" from "uint8[]", or "string" from "string<=10[<=5]".
    Returns type unchanged if not a collection or bounded type.
    """
    if "["  in typename: typename = typename[:typename.index("[")]
    if "<=" in typename: typename = typename[:typename.index("<=")]
    return typename


def time_message(val, to_message=True, clock_type=None):
    """
    Converts ROS2 time/duration between `rclpy` and `builtin_interfaces` objects.

    @param   val         ROS2 time/duration object from `rclpy` or `builtin_interfaces`
    @param   to_message  whether to convert from `rclpy` to `builtin_interfaces` or vice versa
    @param   clock_type  ClockType for converting to `rclpy.Time`, defaults to `ROS_TIME`
    @return              value converted to appropriate type, or original value if not convertible
    """
    to_message, clock_type = bool(to_message), (clock_type or rclpy.clock.ClockType.ROS_TIME)
    if isinstance(val, tuple(ROS_TIME_CLASSES)):
        rcl_cls = next(k for k, v in ROS_TIME_MESSAGES.items() if isinstance(val, (k, v)))
        is_rcl = isinstance(val, tuple(ROS_TIME_MESSAGES))
        name = "to_msg" if to_message and is_rcl else "from_msg" if to_message == is_rcl else None
        args = [val] + ([clock_type] if rcl_cls is rclpy.time.Time and "from_msg" == name else [])
        return getattr(rcl_cls, name)(*args) if name else val
    return val


def to_nsec(val):
    """Returns value in nanoseconds if value is ROS2 time/duration, else value."""
    if not isinstance(val, tuple(ROS_TIME_CLASSES)):
        return val
    if hasattr(val, "nanoseconds"):  # rclpy.Time/Duration
        return val.nanoseconds
    return val.sec * 10**9 + val.nanosec  # builtin_interfaces.msg.Time/Duration


def to_sec(val):
    """Returns value in seconds if value is ROS2 time/duration, else value."""
    if not isinstance(val, tuple(ROS_TIME_CLASSES)):
        return val
    if hasattr(val, "nanoseconds"):  # rclpy.Time/Duration
        secs, nsecs = divmod(val.nanoseconds, 10**9)
        return secs + nsecs / 1E9
    return val.sec + val.nanosec / 1E9  # builtin_interfaces.msg.Time/Duration


def to_sec_nsec(val):
    """Returns value as (seconds, nanoseconds) if value is ROS2 time/duration, else value."""
    if not isinstance(val, tuple(ROS_TIME_CLASSES)):
        return val
    if hasattr(val, "seconds_nanoseconds"):  # rclpy.Time
        return val.seconds_nanoseconds()
    if hasattr(val, "nanoseconds"):  # rclpy.Duration
        return divmod(val.nanoseconds, 10**9)
    return (val.sec, val.nanosec)  # builtin_interfaces.msg.Time/Duration


__all__ = [
    "AnyMsg", "ROSLogHandler", "DDS_TYPES", "FAMILY", "PARAM_SEPARATOR",
    "PRIVATE_PREFIX", "ROS_ALIAS_TYPES", "ROS_TIME_CLASSES", "ROS_TIME_TYPES",
    "canonical", "create_client", "create_publisher", "create_rate", "create_service",
    "create_subscriber", "create_timer", "delete_param", "deserialize_message",
    "destroy_entity", "format_param_name", "get_logger", "get_message_class",
    "get_message_definition", "get_message_fields", "get_message_type",
    "get_message_type_hash", "get_message_value", "get_namespace", "get_node_name",
    "get_nodes", "get_package_share_directory", "get_param", "get_param_names",
    "get_params", "get_rostime", "get_service_definition", "get_service_request_class",
    "get_service_response_class", "get_services", "get_topic_qos", "get_topics",
    "has_param", "init_node", "init_params", "is_ros_message", "is_ros_service",
    "is_ros_time", "make_duration", "make_time", "ok", "register_init", "remap_name",
    "resolve_name", "scalar", "serialize_message", "set_param", "shutdown", "spin",
    "spin_once", "spin_until_future_complete", "start_spin", "time_message", "to_nsec",
    "to_sec", "to_sec_nsec"
]
