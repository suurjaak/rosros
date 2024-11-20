# -*- coding: utf-8 -*-
"""
Simple unified interface to ROS1 / ROS2.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     11.02.2022
@modified    20.11.2024
------------------------------------------------------------------------------
"""
## @namespace rosros
__title__        = "rosros"
__version__      = "0.2.6"
__version_info__ = (0, 2, 6)

from rosros import api
from rosros import rclify
from rosros import rospify
from rosros import util
from rosros.core import *
