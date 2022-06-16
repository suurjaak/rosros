"""
Stand-ins for `rospy.timer` in ROS2.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     30.05.2022
@modified    30.05.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rospify.timer
import abc
import threading
import time

import rclpy.clock
import rclpy.timer

from .. import ros2
from . import exceptions


def sleep(duration):
    """
    Sleeps for the specified duration in ROS time.

    @param   duration   seconds (or Duration) to sleep, <=0 returns immediately

    @throws  ROSInterruptException           if ROS shutdown occurs before sleep completes
    @throws  ROSTimeMovedBackwardsException  if ROS time is set backwards
    """
    jumpback = None
    timer = ros2.create_timer(ros2.to_sec(duration), callback=None)

    def jumphandler(jump):
        nonlocal jumpback
        jumpback = jump
        ros2.destroy_entity(rate)

    thresh = rclpy.clock.JumpThreshold(min_forward=None, min_backward=ros2.make_duration(nsecs=-1))
    handle = ros2.NODE.get_clock().create_jump_callback(thresh, post_callback=jumphandler)
    rate = rclpy.timer.Rate(timer, context=ros2.NODE.context)
    try:
        rate.sleep()
    except rclpy.exceptions.ROSInterruptException as e:
        raise exceptions.ROSInterruptException(*e.args) from e
    finally:
        handle.unregister()
        ros2.destroy_entity(timer)
    if jumpback:
        raise exceptions.ROSTimeMovedBackwardsException(ros2.to_sec(jumpback.delta))



class Rate(abc.ABC):
    """Stand-in for `rospy.Rate`, wrapping the creation of ROS2 rate instance."""

    def __new__(cls, hz, reset=None):
        """
        Returns rclpy.timer.Rate instance.

        @param   hz     rate to determine sleeping
        @param   reset  ignored (ROS1 compatibility stand-in)
        """
        return ros2.create_rate(hz)

    @classmethod
    def __subclasshook__(cls, C):
        """Returns true if C is a ROS2 rate class, else `NotImplemented`."""
        return True if issubclass(C, rclpy.timer.Rate) else NotImplemented


class TimerEvent:
    """
    Constructor.

    Port of rospy.timer.TimerEvent from ROS1 to ROS2.

    @param   last_expected     when previous callback should have happened, as rospy.Time
    @param   last_real         when previous callback actually happened, as rospy.Time
    @param   current_expected  when current callback should have been called, as rospy.Time
    @param   last_duration     duration of the last callback (end time minus start time) in seconds,
                               as float. Note that this is always in wall-clock time.
    """
    def __init__(self, last_expected, last_real, current_expected, current_real, last_duration):
        self.last_expected    = last_expected
        self.last_real        = last_real
        self.current_expected = current_expected
        self.current_real     = current_real
        self.last_duration    = last_duration


class Timer(abc.ABC):
    """Stand-in for `rospy.Timer`, wrapping the creation of ROS2 timer instance."""

    def __new__(cls, period, callback, oneshot=False, reset=None):
        """
        Returns rclpy.timer.Timer instance.

        @param   period    desired period between callbacks, as `Duration`
        @param   callback  function to be called, taking rospy.TimerEvent
        @param   oneshot   if True, fire only once, otherwise fire continuously
                           until shutdown is called [default: False]
        @param   reset     ignored (ROS1 compatibility stand-in)
        """
        if callable(callback):
            def callwrapper(period, callback):
                """Closure for timer callback, retains state for populating TimerEvent."""
                if isinstance(period, (int, float)): period = ros2.make_duration(period)
                last_expected, last_real, last_duration = None, None, None
                current_expected = ros2.get_rostime() + period

                def inner():
                    """Feeds TimerEvent to callback, expected in rospy API."""
                    nonlocal current_expected, last_expected, last_real, last_duration
                    current_real = ros2.get_rostime()
                    start = time.time()
                    callback(TimerEvent(last_expected, last_real, current_expected, current_real,
                                        last_duration))
                    last_duration = time.time() - start
                    last_expected, last_real = current_expected, current_real
                    current_expected += period
                return inner
            callback = callwrapper(period, callback)
        return ros2.create_timer(period, callback, oneshot)

    @classmethod
    def __subclasshook__(cls, C):
        """Returns true if C is a ROS2 timer class, else `NotImplemented`."""
        return True if issubclass(C, (rclpy.timer.Timer, threading.Thread)) else NotImplemented



__all__ = [
    "sleep", "Rate", "Timer"
]
