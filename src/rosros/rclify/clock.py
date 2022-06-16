"""
Partial stand-in for `rclpy.clock` in ROS1, providing `ClockType` and `Clock`.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     14.02.2022
@modified    15.04.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rclify.clock
from enum import IntEnum
import time

import rospy

# from .. import patch  # Imported late to avoid circular import


class ClockType(IntEnum):
    """Enum for clock type."""

    ## Time according to ROS (may be simulated time)
    ROS_TIME = 1

    ## Time according to system clock (wall time)
    SYSTEM_TIME = 2

    ## Monotonously increasing time
    STEADY_TIME = 3


class Clock:
    """Simple clock interface mimicking `rclpy.clock.Clock`, only providing `now()`."""

    def __init__(self, *, clock_type=ClockType.SYSTEM_TIME):
        """Raises error if unknown clock type."""
        if not isinstance(clock_type, ClockType):
            raise TypeError("Clock type must be a ClockType enum")
        self.clock_type = clock_type

    def __repr__(self):
        return "Clock(clock_type={0})".format(self.clock_type.name)

    def now(self):
        """Returns `rospy.Time` according to clock type."""
        if   ClockType.ROS_TIME    == self.clock_type:
            result = rospy.get_rostime()
        elif ClockType.STEADY_TIME == self.clock_type:
            result = rospy.Time(nsecs=time.monotonic_ns())
        else:  # ClockType.SYSTEM_TIME
            result = rospy.Time(nsecs=time.time_ns())
        from .. import patch  # Imported late to avoid circular import
        patch.set_extra_attribute(result, "_clock_type", self.clock_type)
        return result


__all__ = ["ClockType", "Clock"]
