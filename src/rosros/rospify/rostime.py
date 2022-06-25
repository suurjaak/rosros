"""
Stand-ins for `rospy.rostime` in ROS2.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     30.05.2022
@modified    25.06.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rospify.rostime
from .. import ros2

import rclpy.duration
import rclpy.time

## Stand-in for `rospy.Duration`
Duration = rclpy.duration.Duration

## Stand-in for `rospy.Time`
Time = rclpy.time.Time


def get_rostime():
    """Returns current ROS time as `rclpy.Time`."""
    return ros2.get_rostime()


def get_time():
    """Returns the current ROS time as UNIX timestamp float."""
    return ros2.to_sec(ros2.get_rostime())


__all__ = [
    "Duration", "Time", "get_rostime", "get_time"
]
