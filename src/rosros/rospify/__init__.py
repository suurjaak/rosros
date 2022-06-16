"""
Exposes local ported ROS1 API for ROS2, or imports rospy if ROS1 environment.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     06.03.2022
@modified    30.05.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rospify
import os


if os.getenv("ROS_VERSION") != "2":
    from rospy import *
else:
    from rosros.rospify.rospify import *
