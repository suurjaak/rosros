"""
Stand-in for `rclpy.time`, provides `rospy.Duration`.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     14.02.2022
@modified    23.02.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rclify.time
from rospy import Time

CONVERSION_CONSTANT = 10 ** 9
