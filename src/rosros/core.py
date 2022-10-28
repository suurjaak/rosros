# -*- coding: utf-8 -*-
"""
ROS system and node functionality.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     11.02.2022
@modified    28.10.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.core
import os

ros1 = ros2 = None
if os.getenv("ROS_VERSION") != "2":
    from . import ros1
else:
    from . import ros2
ros = ros1 or ros2


## rospy.AnyMsg in ROS1, equivalent stand-in in ROS2
AnyMsg = ros.AnyMsg

## rosbag.Bag in ROS1 (with some additional functionality), equivalent class in ROS2
Bag = ros.Bag


def init_node(name, args=None, namespace=None, anonymous=False, log_level=None, enable_rosout=True,
              multithreaded=True, reentrant=False):
    """
    Initializes ROS and creates ROS node.

    @param   name           node name, without namespace
    @param   args           list of command-line arguments for the node
    @param   namespace      node namespace override
    @param   anonymous      whether to auto-generate a unique name for the node,
                            using the given name as base
    @param   log_level      level to set for ROS logging
                            (name like "DEBUG" or one of `logging` constants like `logging.DEBUG`)
    @param   enable_rosout  `False` to suppress auto-publication of rosout
    @param   multithreaded  ROS2 only, ignored in ROS1:
                            use `MultiThreadedExecutor` instead of `SingleThreadedExecutor`
    @param   reentrant      ROS2 only, ignored in ROS1:
                            use `ReentrantCallbackGroup` instead of `MutuallyExclusiveCallbackGroup`
    """
    if ros1: ros1.init_node(name, args, namespace, anonymous, log_level, enable_rosout)
    else: ros2.init_node(name, args, namespace, anonymous, log_level, multithreaded, reentrant)


def init_params(defaults=None, **defaultkws):
    """
    Sets all parameters on node from defaults dictionary.

    In ROS2, auto-declares unregistered parameters.

    @param   defaults    nested dictionary with suitable keys and values for ROS
                         (keys must be valid namespace names,
                          list values must not contain nested values)
    @param   defaultkws  parameters as key=value
    @return              full nested parameters dictionary, combined from given defaults
                         and externally set parameters on the node
    """
    return ros.init_params(defaults, **defaultkws)


def delete_param(name):
    """
    Deletes parameter from the node.

    @param   name      full name of the parameter in node namespace
    @throws  KeyError  if parameter not set
    """
    ros.delete_param(name)


def has_param(name):
    """
    Returns whether the parameter exists on the node.

    @param   name  full name of the parameter in node namespace
    """
    return ros.has_param(name)


def get_param_names():
    """Returns the names of all parameters on the node."""
    return ros.get_params()


def get_params(nested=True):
    """
    Returns the current ROS node parameters, by default as nested dictionary.

    @param   nested  return a nested dictionary,
                     like `{"my": {"name": 1}}` vs {"my.name": 1}
    """
    return ros.get_params(nested)


def get_param(name, default=None, autoset=True):
    """
    Returns parameter value from the node.

    @param   default   optional default to return if parameter unknown
    @param   autoset   set default value to node as parameter value if unknown
    @return            parameter value, or default if parameter was unknown

    @throws  KeyError  if parameter not set and default not given
    """
    return ros.get_param(name, default, autoset)


def set_param(name, value, descriptor=None):
    """
    Sets a parameter on the node.

    In ROS2, parameter will be auto-declared if unknown so far.

    @param    name        full name of the parameter in node namespace
    @param    value       parameter value to set
    @param    descriptor  optional `rcl_interfaces.msg.ParameterDescriptor` in ROS2
    @return               the set value
    """
    if ros1: return ros1.set_param(name, value)
    return ros2.set_param(name, value, descriptor)


def start_spin():
    """Sets ROS node spinning forever in a background thread."""
    ros.start_spin()


def spin():
    """Spins ROS node forever."""
    ros.spin()


def spin_once(timeout=None):
    """
    Waits until timeout in ROS1; executes one ROS operation or waits until timeout in ROS2.

    @param  timeout  time to wait at most, as seconds or ROS duration;
                     None or <0 waits forever
    """
    ros.spin_once(timeout)


def spin_until_future_complete(future, timeout=None):
    """
    Spins ROS until future complete or timeout reached or ROS shut down.

    @param  future       object with `asyncio.Future`-conforming interface to complete
    @param  timeout      time to wait, as seconds or ROS1 duration
    """
    ros.spin_until_future_complete(future, timeout)


def ok():
    """Returns whether ROS has been initialized and is not shut down."""
    return ros.ok()


def shutdown():
    """Shuts down live ROS node, if any."""
    ros.shutdown()


def create_publisher(topic, cls_or_typename, latch=False, queue_size=0, **qosargs):
    """
    Returns a ROS publisher instance.

    @param   topic            name of topic to open
    @param   cls_or_typename  ROS message class object like `std_msgs.msg.Bool`
                              or message type name like "std_msgs/Bool"
    @param   latch            provide last published message to new subscribers
                              (sets `DurabilityPolicy.TRANSIENT_LOCAL` in ROS2)
    @param   queue_size       queue size of outgoing messages (0 or None: infinite)
                              (sets QoS `depth` in ROS2)
    @param   qosargs          additional key-value arguments for ROS2
                              `QoSProfile`, like `reliability` (uses only `depth`
                              and `durability`=TRANSIENT_LOCAL in ROS1)
    @return                   `rospy.Publisher` or `rclpy.publisher.Publisher`,
                              either will support keyword arguments in `publish()`
                              and have `get_num_connections()`
    """
    if ros1:
        queue_size = qosargs.get("depth", queue_size)
        if qosargs.get("durability") == 1:  # DurabilityPolicy.TRANSIENT_LOCAL
            latch = True
        return ros1.create_publisher(topic, cls_or_typename, latch, queue_size)
    return ros2.create_publisher(topic, cls_or_typename, latch, queue_size, **qosargs)


def create_subscriber(topic, cls_or_typename, callback, callback_args=None,
                      queue_size=0, raw=False, **qosargs):
    """
    Returns a ROS subscriber instance.

    @param   topic            name of topic to listen to
    @param   cls_or_typename  ROS message class object like `std_msgs.msg.Bool`
                              or message type name like "std_msgs/Bool"
    @param   callback         callback function, invoked with received message,
                              and with additional arguments if given
    @param   callback_args    additional arguments to pass to the callback, if any,
                              invoked as `callback(msg, callback_args)´
    @param   queue_size       queue size of incoming messages (0 or None: infinite)
    @param   raw              invoke callback with serialized message bytes,
                              make subscription with AnyMsg in ROS1
    @param   qosargs          additional key-value arguments for ROS2
                              `QoSProfile`, like `reliability` (uses only `depth` in ROS1).
                              `__autodetect` will look up current publishers on the topic
                              and create a compatible QoS.
    @return                   `rospy.Subscriber` or `rclpy.subscription.Subscription`
    """
    if ros1:
        queue_size = qosargs.get("depth", queue_size)
        return ros1.create_subscriber(topic, cls_or_typename, callback, callback_args,
                                      queue_size, raw)
    return ros2.create_subscriber(topic, cls_or_typename, callback, callback_args,
                                  queue_size, raw, **qosargs)


def create_service(service, cls_or_typename, callback, **qosargs):
    """
    Returns a ROS service server instance, for providing a service.

    @param   service          name of service to provide
    @param   cls_or_typename  ROS service class object like `std_srvs.srv.SetBool`
                              or service type name like `"std_srvs/SetBool"
    @param   callback         callback function, invoked with `(request, response)`,
                              expected to return the populated response,
                              or a list/tuple/dictionary for populating the response.
                              If the function only takes one argument,
                              it is invoked with `(request)`.
    @param   qosargs          additional key-value arguments for ROS2
                              `QoSProfile`, like `reliability` (ignored in ROS1)
    @return                   `rospy.Service` or `rclpy.service.Service`
    """
    if ros1: return ros1.create_service(service, cls_or_typename, callback)
    return ros2.create_service(service, cls_or_typename, callback, **qosargs)


def create_client(service, cls_or_typename, **qosargs):
    """
    Returns a ROS service client instance, for invoking a service.

    @param   service          name of service to invoke
    @param   cls_or_typename  ROS service class object like `std_srvs.srv.SetBool`
                              or service type name like `"std_srvs/SetBool"
    @param   qosargs          additional key-value arguments for ROS2
                              `QoSProfile`, like `reliability` (ignored in ROS1)
    @return                   `rospy.ServiceProxy` or `rclpy.client.Client`,
                              either will have `call_async()` returning a future,
                              either will support keyword arguments in calls
                              and be callable itself
    """
    if ros1: return ros1.create_client(service, cls_or_typename)
    return ros2.create_client(service, cls_or_typename, **qosargs)


def create_timer(period, callback, oneshot=False, immediate=False):
    """
    Returns a ROS timer instance.

    @param   period     desired period between callbacks, as seconds or ROS duration
    @param   callback   callback function invoked on timer, with no arguments
    @param   oneshot    whether to fire only once
    @param   immediate  whether to fire once immediately instead of waiting one period
    @return             `rospy.Timer` or `rclpy.timer.Timer`
    """
    if ros1: return ros1.create_timer(period, callback, oneshot, immediate)
    return ros2.create_timer(period, callback, oneshot, immediate)


def create_rate(frequency):
    """
    Returns a ROS rate instance, for sleeping at a fixed rate.

    @param   frequency  rate to sleep at, in Hz
    @return             `rospy.Rate` or `rclpy.rate.Rate`
    """
    return ros.create_rate(frequency)


def destroy_entity(item):
    """Closes the given publisher, subscriber, service client, or service server instance."""
    return ros.destroy_entity(item)


def get_logger():
    """
    Returns `logging.Logger` for logging to ROS log handler.

    Logging methods on the logger (`debug()`, `info()`, etc) accept additional keyword arguments:
    - `__once__`:                whether to log only once from call site
    - `__throttle__`:            seconds to skip logging from call site for
    - `__throttle_identical__`:  whether to skip logging identical consecutive texts from call site
                                 (given log message excluding formatting arguments).
                                 Combines with `__throttle__` to skip duplicates for a period.
    """
    return ros.get_logger()


def get_rostime():
    """Returns ROS time instance for current ROS clock time."""
    return ros.get_rostime()


def get_namespace():
    """Returns ROS node namespace."""
    return ros.get_namespace()


def get_node_name():
    """Returns ROS node full name with namespace."""
    return ros.get_node_name()


def get_nodes():
    """Returns all ROS nodes, as `[node full name, ]`."""
    return ros.get_nodes()


def get_topics():
    """Returns all available ROS topics, as `[(topic name, [type name, ]), ]`."""
    return ros.get_topics()


def get_services(node=None, namespace=None, include_types=True):
    """
    Returns all available ROS services, as `[(service name, [type name, ]), ]`.

    @param   node           full name of the node to return services for, if any
    @param   namespace      full or partial namespace to scope services from
    @param   include_types  if false, type names will be returned as an empty list
    """
    return ros.get_services(node, namespace, include_types)


def remap_name(name, namespace=None):
    """
    Returns the absolute remapped topic/service name if mapping exists.

    @param   name       name to seek exact remapping for
    @param   namespace  namespace to resolve relative and private names to,
                        by default current node namespace
    @return             remapped resolved name, or original if not mapped
    """
    return ros.remap_name(name, namespace)


def resolve_name(name, namespace=None):
    """
    Returns absolute remapped name, namespaced under current node if relative or private.

    @param   namespace  namespace to use if not current node full name
    """
    return ros.resolve_name(name, namespace)


def register_init(node=None):
    """
    Informs `rosros` of ROS having been initialized outside of `init_node()`.

    @param   node  mandatory `rclpy.node.Node` in ROS2
    """
    ros1.register_init() if ros1 else ros2.register_init(node)


__all__ = [
    "ros1", "ros2", "AnyMsg", "Bag",
    "create_client", "create_publisher", "create_rate", "create_service",
    "create_subscriber", "create_timer", "delete_param", "destroy_entity",
    "get_logger", "get_namespace", "get_node_name", "get_nodes", "get_param",
    "get_param_names", "get_params", "get_rostime", "get_services", "get_topics",
    "has_param", "init_node", "init_params", "ok", "register_init", "remap_name",
    "resolve_name", "set_param", "shutdown", "spin", "spin_once",
    "spin_until_future_complete", "start_spin"
]
