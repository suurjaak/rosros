"""
Provides ROS1 API facade for ROS2, or imports rospy if ROS1 environment.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     06.03.2022
@modified    24.06.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rospify
import os


if os.getenv("ROS_VERSION") != "2":
    from rospy import *
    from rospy import timer
    from rospy import Timer, wait_for_message
else:
    from rosros.rospify.rospify import *
