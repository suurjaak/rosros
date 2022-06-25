"""
Stand-ins for `rospy.core` in ROS2.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     30.05.2022
@modified    23.06.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rospify.core

from .. import ros2
from . import exceptions
from . log import logdebug, logwarn, loginfo, logout, logerr, logfatal, \
                  logdebug_throttle, logwarn_throttle, loginfo_throttle, logerr_throttle, \
                  logfatal_throttle, \
                  logdebug_throttle_identical, logwarn_throttle_identical, \
                  loginfo_throttle_identical, logerr_throttle_identical, \
                  logfatal_throttle_identical, \
                  logdebug_once, logwarn_once, loginfo_once, logerr_once, logfatal_once


def is_shutdown():
    """Returns whether ROS2 has been started and shut down."""
    return ros2.SHUTDOWN


def signal_shutdown(reason):
    """
    Initiates shutdown process.

    @param   reason   ignored (ROS1 compatibility stand-in)
    """
    ros2.shutdown()


def get_node_uri():
    """Returns empty string (ROS1 compatibility stand-in)."""
    return ""


def get_ros_root(required=False, env=None):
    """
    Returns rclpy share directory.

    In ROS1, this returns a path along the lines of "/opt/ros/noetic/share/ros".

    @param   env       ignored (ROS1 compatibility stand-in)
    @param   required  fail with ROSException if not found
    @return            rclpy share directory path

    @throws  ROSException  if `require` is True and result unavailable
    """
    try:
        return ros2.get_package_share_directory("rclpy")
    except KeyError:
        if required:
            raise exceptions.ROSException("ROS root share directory not set.")
    return None


ROSRPC = "rosrpc://"

def parse_rosrpc_uri(uri):
    """
    Utility function for parsing ROS-RPC URIs.

    @param   uri  ROSRPC URI
    @return       (address, port)

    @throws  ParameterInvalid  if uri is not a valid ROSRPC URI
    """
    if uri.startswith(ROSRPC):
        dest_addr = uri[len(ROSRPC):]
    else:
        raise exceptions.ParameterInvalid("Invalid protocol for ROS service URL: %s" % uri)
    try:
        if "/" in dest_addr:
            dest_addr = dest_addr[:dest_addr.find("/")]
        dest_addr, dest_port = dest_addr.split(":")
        dest_port = int(dest_port)
    except Exception as e:
        raise exceptions.ParameterInvalid("ROS service URL is invalid: %s" % uri) from e
    return dest_addr, dest_port


__all__ = [
    "ROSRPC", "get_node_uri", "get_ros_root", "is_shutdown", "parse_rosrpc_uri", "signal_shutdown",
    "logdebug", "logwarn", "loginfo", "logout", "logerr", "logfatal",
    "logdebug_throttle", "logwarn_throttle", "loginfo_throttle", "logerr_throttle",
    "logfatal_throttle",
    "logdebug_throttle_identical", "logwarn_throttle_identical", "loginfo_throttle_identical",
    "logerr_throttle_identical", "logfatal_throttle_identical",
    "logdebug_once", "logwarn_once", "loginfo_once", "logerr_once", "logfatal_once"
]
