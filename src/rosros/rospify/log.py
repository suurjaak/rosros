"""
Stand-in for `rospy` logging functionality in ROS2.

Heavily modified copy from ROS1 `rospy.node`,
at https://github.com/ros/ros_comm (`clients/rospy/src/rospy/core.py`),
released under the BSD License.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     01.03.2022
@modified    06.03.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rospify.log

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
import hashlib
import inspect
import pickle

from .. import ros2


def logdebug(msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, level="debug")

def loginfo(msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, level="info")

def logwarn(msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, level="warn")

def logerr(msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, level="error")

def logfatal(msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, level="critical")


def logdebug_throttle(period, msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, throttle=period, level="debug")

def loginfo_throttle(period, msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, throttle=period, level="info")

def logwarn_throttle(period, msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, throttle=period, level="warn")

def logerr_throttle(period, msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, throttle=period, level="error")

def logfatal_throttle(period, msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, throttle=period, level="critical")


def logdebug_throttle_identical(period, msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, throttle=period, throttle_identical=True,
                 level="debug")

def loginfo_throttle_identical(period, msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, throttle=period, throttle_identical=True,
                 level="info")

def logwarn_throttle_identical(period, msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, throttle=period, throttle_identical=True,
                 level="warn")

def logerr_throttle_identical(period, msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, throttle=period, throttle_identical=True,
                 level="error")

def logfatal_throttle_identical(period, msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, throttle=period, throttle_identical=True,
                 level="critical")


def logdebug_once(msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, once=True, level="debug")

def loginfo_once(msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, once=True, level="info")

def logwarn_once(msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, once=True, level="warn")

def logerr_once(msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, once=True, level="error")

def logfatal_once(msg, *args, **kwargs):
    _base_logger(msg, args, kwargs, once=True, level="critical")


logout = loginfo  # alias deprecated name

logerror = logerr  # alias logerr



def _make_caller_id(depth=2):
    """Returns caller id at specified call stack depth."""
    frame = inspect.currentframe().f_back
    while depth and frame and frame.f_back:
        frame, depth = frame.f_back, depth - 1
    return pickle.dumps((inspect.getabsfile(frame), frame.f_lineno, frame.f_lasti))


def _base_logger(msg, args, kwargs, throttle=None,
                 throttle_identical=False, level=None, once=False):
    """
    Issues the logging call, if throttling or once-argument allow for it.

    @param   msg                 log message
    @param   args                positional arguments for log message
    @param   kwargs              keyword arguments for log message
    @param   throttle            seconds to throttle log messages from call site for
    @param   throttle_identical  whether to throttle identical consecutive log messages
    @param   level               log level name
    @param   once                whether to log only once from callsite
    """
    do_log = True
    if once:
        do_log = LogInhibitor.passes_once(_make_caller_id())
    elif throttle_identical:
        caller_id, throttle_elapsed = _make_caller_id(), False
        if throttle is not None:
            throttle_elapsed = LogInhibitor.passes_throttle(caller_id, throttle)
        do_log = LogInhibitor.passes_identical(caller_id, msg) or throttle_elapsed
    elif throttle:
        do_log = LogInhibitor.passes_throttle(_make_caller_id(), throttle)

    if do_log: getattr(ros2.get_logger(), level.lower())(msg, *args, **kwargs)


class LogInhibitor:

    ## Caller IDs registered for logxyz_once()
    ONCES = set()

    ## Caller IDs and last timestamps for logxyz_throttle() and logxyz_throttle_identical()
    TIMES = {}

    ## Caller IDs and log message hashes for logxyz_throttle_identical()
    HASHES = {}

    @classmethod
    def passes_once(cls, caller_id):
        """Returns whether the caller ID has not been once()-d before."""
        result = caller_id not in cls.ONCES
        cls.ONCES.add(caller_id)
        return result

    @classmethod
    def passes_throttle(cls, caller_id, period):
        """Returns whether time from last throttle() was more than specified seconds ago, if any."""
        now, last = ros2.get_rostime(), cls.TIMES.get(caller_id)
        if last is not None and last > now: cls.TIMES.clear()  # Reset all on time jump backward
        last = cls.TIMES.get(caller_id)
        result = last is None or now - last > ros2.make_duration(period)
        cls.TIMES[caller_id] = now
        return result

    @classmethod
    def passes_identical(cls, caller_id, msg):
        """Returns whether last message from caller was different, if any."""
        result, msg_hash = False, hashlib.md5(msg.encode()).hexdigest()
        if msg_hash != cls.HASHES.get(caller_id):
            cls.HASHES[caller_id] = msg_hash
            result = True
        return result


__all__ = [
    "logdebug", "loginfo", "logout", "logwarn", "logerr", "logfatal",
    "logdebug_throttle", "loginfo_throttle", "logwarn_throttle",
    "logerr_throttle", "logfatal_throttle",
    "logdebug_throttle_identical", "loginfo_throttle_identical",
    "logwarn_throttle_identical", "logerr_throttle_identical", "logfatal_throttle_identical",
    "logdebug_once", "loginfo_once", "logwarn_once", "logerr_once", "logfatal_once",
]
