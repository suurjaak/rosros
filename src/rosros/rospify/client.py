"""
Stand-ins for `rospy.client` in ROS2.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     30.05.2022
@modified    19.06.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rospify.client
import logging
import re
import sys

import rclpy.exceptions
import rclpy.timer

from .. import ros2
from .. import util
from . import exceptions
from . msproxy import MasterProxy
from . names  import get_name, get_namespace


# Log levels corresponding to rosgraph_msgs.Log constants
DEBUG =  1
INFO  =  2
WARN  =  4
ERROR =  8
FATAL = 16


## Map rospy log level constants to Python logging level constants
_ROSPY_LOG_LEVEL_TO_PY_LEVEL = {
    DEBUG: logging.DEBUG,
    INFO:  logging.INFO,
    WARN:  logging.WARN,
    ERROR: logging.ERROR,
    FATAL: logging.FATAL,
}


# class to distinguish whether or not user has passed us a default value
class _unspecified: pass


def delete_param(param_name):
    """
    Deletes a parameter on the node.

    @param   param_name    parameter name

    @throws  KeyError      if parameter is not set
    """
    ros2.delete_param(param_name)


def has_param(param_name):
    """
    Test if parameter exists on the node.

    @param   param_name    parameter name
    """
    return ros2.has_param(param_name)


def get_master(env=None):
    """
    Returns a partial stand-in of `rospy.MasterProxy`.

    @param   env  ignored (ROS1 compatbility stand-in)
    """
    return MasterProxy(ros2.NODE)


def get_param(param_name, default=_unspecified):
    """
    Returns a parameter value from the node.

    @param   default       default value to return if key is not set
    @return                parameter value

    @throws  KeyError      if value not set and default is not given
    """
    if default is _unspecified and not ros2.has_param(param_name):
        raise KeyError(param_name)
    return ros2.get_param(param_name, default)


## Stand-in for `rospy.get_param_cached`
get_param_cached = get_param


def get_param_names():
    """Returns a list of node parameter names, with absolute namespace."""
    ns = util.namejoin(get_namespace(), get_name())
    return [util.namejoin(ns, x) for x in ros2.get_param_names()]


def get_published_topics(namespace="/"):
    """
    Returns a list of published topics.

    @return   list of topic and type names, as [[topic, type], ]

    @throws   ROSException  if retrieving topics fails
    """
    try:
        return MasterProxy(ros2.NODE).getPublishedTopics(namespace)[-1]
    except Exception as e:
        raise exceptions.ROSException("unable to get published topics") from e


def init_node(name, argv=None, anonymous=False, log_level=None,
              disable_rostime=None, disable_rosout=False,
              disable_signals=None, xmlrpc_port=None, tcpros_port=None):
    """
    Inits ROS2 and creates ROS2 node.


    @param   name              node name, with no namespace
    @param   argv              list of command-line arguments for the node
    @param   anonymous         whether to auto-generate a unique name for the node,
                               using the given name as base
    @param   log_level         log level to set for the node logger,
                               one of `rospify.DEBUG .INFO .WARN .ERROR .FATAL`
    @param   disable_rostime   ignored (ROS1 compatibility stand-in)
    @param   disable_rosout    whether to suppress auto-publication of rosout
    @param   disable_signals   ignored (ROS1 compatibility stand-in)
    @param   xmlrpc_port       ignored (ROS1 compatibility stand-in)
    @param   tcpros_port       ignored (ROS1 compatibility stand-in)

    @throws  ROSInitException  if initialization/registration fails
    @throws  ValueError        if name is invalid
    """
    if not name:
        raise ValueError("name must not be empty")
    if "/" in name:
        raise ValueError("namespaces are not allowed in node names")

    log_level = _ROSPY_LOG_LEVEL_TO_PY_LEVEL.get(log_level, log_level)
    try: ros2.init_node(name, argv, anonymous=anonymous, log_level=log_level,
                        enable_rosout=not disable_rosout)
    except Exception as e:
        raise exceptions.ROSInitException() from e


def myargv(argv=None):
    """
    Returns command-line arguments with remappings removed.

    @param   argv  arguments to use if not `sys.argv`
    @return        list of arguments with remappings removed
    """
    REMAP_RGX = re.compile(r"^([\~\/a-z]|_|__)[\w\/]*:=.*", re.I)
    argv = list(sys.argv if argv is None else argv)
    while "--remap" in argv:
        idx = argv.index("--remap")
        argv.pop(idx)
        while len(argv) > idx and REMAP_RGX.match(argv[idx]):
            argv.pop(idx)
    return argv


def on_shutdown(h):
    """Registers function to be called on shutdown, after node has been torn down."""

    # ROS2 requires that the callback be a bound method
    class Cls:
        def __init__(self, func):
            self.func = func
        def callback(self):
            return self.func()

    ros2.NODE.context.on_shutdown(Cls(h).callback)


def search_param(param_name):
    """
    Search for a parameter on the node.

    @param   param_name  parameter name
    @return              key of matching parameter or None if no matching parameter
    """
    ns = util.namejoin(get_namespace(), get_name())
    if param_name.startswith(ns):
        param_name = param_name[len(ns):]
    return util.namejoin(ns, param_name) if param_name in ros2.get_param_names() else None


def set_param(param_name, param_value):
    """
    Set a parameter on the node.

    If param_value is a dictionary, it will be treated as a parameter tree,
    where param_name is the namespace. For example:::
      {"x": 1, "y": 2, "sub": {"z": 3}}
    will set param_name.x=1, param_name.y=2, and param_name.sub.z=3.
    Furthermore, it will replace all existing parameters in the
    param_name namespace with the parameters in param_value. You must
    set parameters individually if you wish to perform a union update.

    @param   param_name    parameter name
    @param   param_value   parameter value

    @throws  ROSException  if setting parameter failed
    """
    try:
        if isinstance(param_value, dict):
            ns = "%s." % param_name.strip(".")
            for n in [x for x in ros2.get_param_names() if x.startswith(ns)]:
                ros2.delete_param(n)
            for k, v in util.flatten_dict({param_name: param_value}).items():
                ros2.set_param(k, v)
        else: ros2.set_param(param_name, param_value)
    except Exception as e:
        raise exceptions.ROSException() from e


def spin():
    """Spins ROS2 node forever."""
    ros2.spin()


def wait_for_message(topic, topic_type, timeout=None):
    """
    Receive one message from topic.

    This will create a new subscription to the topic, receive one message, then unsubscribe.

    @param   topic       name of topic
    @param   topic_type  topic type class
    @param   timeout     timeout time in seconds or ROS Duration
    @return              ROS message

    @throws  ROSException           if specified timeout is exceeded
    @throws  ROSInterruptException  if shutdown interrupts wait
    """
    timeout = ros2.to_sec(timeout) if timeout is not None else 2**31 - 1
    sub, timer, rate = None, None, None

    try:
        def on_message(msg):
            msgs.append(msg)
            ros2.destroy_entity(rate)

        msgs = []
        timer = ros2.create_timer(timeout, callback=None)
        rate = rclpy.timer.Rate(timer, context=ros2.NODE.context)
        sub = ros2.create_subscriber(topic, topic_type, on_message)
        if not msgs: rate.sleep()
        if not msgs:
            err = "timeout exceeded while waiting for message on topic %s" % topic
            raise exceptions.ROSException(err)
    except rclpy.exceptions.ROSInterruptException as e:
        raise exceptions.ROSInterruptException(*e.args) from e
    finally:
        ros2.destroy_entity(timer)
        ros2.destroy_entity(sub)


__all__ = [
    "delete_param", "get_master", "get_param", "get_param_cached", "get_param_names",
    "get_published_topics", "has_param", "init_node", "myargv", "on_shutdown",
    "search_param", "set_param", "spin", "wait_for_message"
]
