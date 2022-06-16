"""
Stand-in for `rclpy.duration`, provides `rospy.Duration`.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     17.02.2022
@modified    16.06.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rclify.duration
from rospy import Duration

__all__ = ["Duration"]
