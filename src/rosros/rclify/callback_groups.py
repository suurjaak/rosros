"""
Partial stand-in for `rclpy.callback_groups` in ROS1, providing `CallbackGroup` classes.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     12.06.2022
@modified    12.06.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rclify.callback_groups
import weakref


class CallbackGroup:
    """
    The base class for a callback group.

    A callback group controls when callbacks are allowed to be executed.

    This class should not be instantiated.
    Instead, classes should extend it and implement :meth:`can_execute`,
    :meth:`beginning_execution`, and :meth:`ending_execution`.
    """

    def __init__(self):
        self.entities = set()

    def add_entity(self, entity):
        """Adds an entity to this group."""
        self.entities.add(weakref.ref(entity))

    def has_entity(self, entity):
        """Returns whether an entity has been added to this group."""
        return weakref.ref(entity) in self.entities

    def can_execute(self, entity):
        """Returns True (ROS2 compatibility stand-in)."""
        return True

    def beginning_execution(self, entity):
        """Returns True (ROS2 compatibility stand-in)."""
        return True

    def ending_execution(self, entity):
        """Does nothing (ROS2 compatibility stand-in)."""


class MutuallyExclusiveCallbackGroup(CallbackGroup):
    """API stand-in for ROS2 compatibility."""

class ReentrantCallbackGroup(CallbackGroup):
    """API stand-in for ROS2 compatibility."""


__all__ = ["CallbackGroup", "MutuallyExclusiveCallbackGroup", "ReentrantCallbackGroup"]
