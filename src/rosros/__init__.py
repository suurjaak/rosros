# -*- coding: utf-8 -*-
"""
Simple unified interface to ROS1 / ROS2.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     11.02.2022
@modified    30.01.2024
------------------------------------------------------------------------------
"""
## @namespace rosros
__title__        = "rosros"
__version__      = "0.2.5"
__version_info__ = (0, 2, 5)

from rosros import api
from rosros import rclify
from rosros import rospify
from rosros import util
from rosros.core import *
