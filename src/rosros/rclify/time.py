"""
Stand-in for `rclpy.time`, provides `rospy.Duration`.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     14.02.2022
@modified    16.06.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rclify.time
from rospy import Time

CONVERSION_CONSTANT = 10 ** 9


__all__ = ["CONVERSION_CONSTANT", "Time"]
