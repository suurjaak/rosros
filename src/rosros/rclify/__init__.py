"""
Provides ROS2 API facade for ROS1, or imports rclpy if ROS2 environment.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     17.02.2022
@modified    23.06.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rclify
import os


if os.getenv("ROS_VERSION") != "2":
    from rosros.rclify import callback_groups
    from rosros.rclify import clock
    from rosros.rclify import duration
    from rosros.rclify import exceptions
    from rosros.rclify import executors
    from rosros.rclify import node
    from rosros.rclify import parameter
    from rosros.rclify import qos
    from rosros.rclify import task
    from rosros.rclify import time
    from rosros.rclify import topic_endpoint_info
    from rosros.rclify.parameter import Parameter
    from rosros.rclify.task import Future
    from rosros.rclify.rclify import *
else:
    from rclpy import *
    import rclpy.clock as clock
    import rclpy.duration
    import rclpy.exceptions
    import rclpy.node
    import rclpy.parameter
    import rclpy.qos
    import rclpy.task
    import rclpy.time
    import rclpy.topic_endpoint_info
