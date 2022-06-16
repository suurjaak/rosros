"""
Copy of ROS1 `rospy.exceptions` for ROS2, plus a few other rospy exceptions.

Supplemented copy from ROS1 `rospy.exceptions`,
at https://github.com/ros/ros_comm (`clients/rospy/src/rospy/exceptions.py`),
released under the BSD License.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     01.03.2022
@modified    07.03.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rospify.exceptions

# Original file copyright notice:

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


class ROSException(Exception):
    """Base exception class for ROS clients"""

class ROSSerializationException(ROSException):
    """Exception for message serialization errors"""

class ROSInitException(ROSException):
    """Exception for errors initializing ROS state"""

class ROSInterruptException(ROSException, KeyboardInterrupt):
    """
    Exception for operations that interrupted, e.g. due to shutdown.

    This is a subclass of both ROSException and KeyboardInterrupt
    so that it can be used in logic that expects either.
    """

class ROSTimeMovedBackwardsException(ROSInterruptException):
    """Exception if time moved backwards"""

    def __init__(self, time):
        self.time = time
        """The amount of time in seconds."""
        super().__init__("ROS time moved backwards")

class ROSInternalException(Exception):
    """Base class for exceptions that are internal to the ROS system"""

class TransportException(ROSInternalException):
    """Base class for transport-related exceptions"""

class TransportTerminated(TransportException):
    """Internal class for representing broken connections"""

class TransportInitError(TransportException):
    """Internal exception for representing exceptions that occur establishing transports"""

class ParameterInvalid(Exception):
    """Exception that is raised when a parameter fails validation checks"""
    def __init__(self, message):
        self._message = message

    def __str__(self):
        return str(self._message)
