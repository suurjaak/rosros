"""
Stand-in for `rclpy` in ROS1.

Heavily modified copy from ROS2 `rclpy`,
at https://github.com/ros2/rclpy (`rclpy/rclpy/__init__.py`),
released under the Apache 2.0 License.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     16.02.2022
@modified    16.06.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rclify.rclify

# Original file copyright notice:
#
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rospy
import rospy.core

#from .. import patch  # Imported late to avoid circular import
from .. import ros1
from . node import Node


def init(*, args=None, context=None):
    """
    Does nothing (ROS2 API compatibility stand-in)

    @param  args     ignored (ROS2 API compatibility stand-in)
    @param  context  ignored (ROS2 API compatibility stand-in)
    """
    from .. import patch  # Imported late to avoid circular import
    patch.patch_ros1_rclify()


def create_node(node_name, *, context=None, cli_args=None, namespace=None,
                use_global_arguments=True, enable_rosout=True, start_parameter_services=True,
                parameter_overrides=None, allow_undeclared_parameters=False,
                automatically_declare_parameters_from_overrides=False
):
    """
    Initializes ROS1 and returns an instance of `rosros.rclify.node.Node`.

    @param   node_name                  a name to give to the node
    @param   context                    ignored (ROS2 API compatibility stand-in)
    @param   cli_args                   command-line arguments to be used by the node
    @param   namespace                  node namespace
    @param   use_global_arguments       should the node use process-wide command-line arguments
    @param   enable_rosout              publish logging to `/rosout`
    @param   start_parameter_services   ignored (ROS2 API compatibility stand-in)
    @param   parameter_overrides        list of `Parameter` to override the
                                        initial values of parameters declared on the node
    @param allow_undeclared_parameters  allow undeclared parameters;
                                        this option doesn't affect `parameter_overrides`
    @param automatically_declare_parameters_from_overrides
                                        implicitly declare parameters from `parameter_overrides`
    @return                             `rosros.rclify.node.Node`
    """
    return Node(
        node_name, context=context, cli_args=cli_args, namespace=namespace,
        use_global_arguments=use_global_arguments,
        enable_rosout=enable_rosout,
        start_parameter_services=start_parameter_services,
        parameter_overrides=parameter_overrides,
        allow_undeclared_parameters=allow_undeclared_parameters,
        automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides
    )


def spin_once(node=None, *, executor=None, timeout_sec=None):
    """
    Waits until timeout or ROS shut down, or forever if timeout not specified.

    @param  node         ignored (ROS2 API compatibility stand-in)
    @param  executor     ignored (ROS2 API compatibility stand-in)
    @param  timeout_sec  time to wait, as seconds or ROS1 duration
    """
    ros1.spin_once(timeout_sec)


def spin(node=None, executor=None):
    """
    Waits forever.

    @param  node         ignored (ROS2 API compatibility stand-in)
    @param  executor     ignored (ROS2 API compatibility stand-in)
    """
    ros1.spin()


def spin_until_future_complete(node=None, future=None, executor=None, timeout_sec=None):
    """
    Waits until future complete or timeout reached or ROS shut down.

    @param  node         ignored (ROS2 API compatibility stand-in)
    @param  future       object with `asyncio.Future`-conforming interface to complete
                         (will not be awaited with `async`)
    @param  executor     ignored (ROS2 API compatibility stand-in)
    @param  timeout_sec  time to wait, as seconds or ROS1 duration
    """
    ros1.spin_until_future_complete(future, timeout_sec)


def get_default_context():
    """Returns None (ROS2 API compatibility stand-in)."""
    return None


def get_global_executor():
    """Returns None (ROS2 API compatibility stand-in)."""
    return None


def get_rmw_implementation_identifier():
    """Returns empty string (ROS2 API compatibility stand-in)."""
    return ""


def ok(*, context=None):
    """
    Returns whether rospy has been initialized and is not shut down.

    @param  context  ignored (ROS2 API compatibility stand-in)
    """
    return ros1.ok()


def shutdown(*, context=None):
    """
    Sends the shutdown signal to rospy.

    @param  context  ignored (ROS2 API compatibility stand-in)
    """
    rospy.signal_shutdown(reason="")


def try_shutdown(*, context=None):
    """
    Sends the shutdown signal to rospy.

    @param  context  ignored (ROS2 API compatibility stand-in)
    """
    rospy.signal_shutdown(reason="")


__all__ = [
    "create_node", "get_default_context", "get_global_executor",
    "get_rmw_implementation_identifier", "init", "ok", "shutdown",
    "spin", "spin_once", "spin_until_future_complete", "try_shutdown"
]
