"""
Partial stand-in for `rclpy.executors` in ROS1, providing `Executor` classes.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     12.06.2022
@modified    12.06.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rclify.executors
import threading

from .. import ros1
from . task import Task



class TimeoutException(Exception):
    """Signal that a timeout occurred."""

class ShutdownException(Exception):
    """Signal that executor was shut down."""

class ExternalShutdownException(Exception):
    """Context has been shutdown."""

class ConditionReachedException(Exception):
    """Future has been completed."""



class Executor:
    """API stand-in for ROS2 compatibility, all spinning is performed by sleep."""

    def __init__(self, *, context=None):
        """
        @param   context  ignored (ROS2 compatibility stand-in)
        """
        super().__init__()
        self._context     = context
        self._nodes       = set()
        self._nodes_lock  = threading.RLock()
        self._is_shutdown = False

    @property
    def context(self):
        """Returns context given in constructor, or None ((ROS2 compatibility stand-in)."""
        return self._context

    def create_task(self, callback, *args, **kwargs):
        """
        Adds a callback or coroutine to be executed during spin, returns a Future.

        Arguments to this function are passed to the callback.

        @param   callback  a callback to be run in the executor
        """
        return Task(callback, args, kwargs, executor=self)

    def shutdown(self, timeout_sec=None):
        """
        Shuts down ROS.

        @param   timeout_sec  ignored (ROS2 compatibility stand-in)
        @return               True
        """
        self._is_shutdown = True
        with self._nodes_lock:
            self._nodes = set()
        ros1.shutdown()
        return True

    def add_node(self, node):
        """
        Adds a node to this executor.

        @param   node  the node to add to the executor
        @return  True if the node was added, False if it already existed
        """
        with self._nodes_lock:
            node.executor = self
            result = node in self._nodes
            self._nodes.add(node)
            return result

    def remove_node(self, node):
        """
        Stop managing this node's callbacks.

        @param   node  the node to remove from the executor
        """
        with self._nodes_lock:
            self._nodes.discard(node)

    def wake(self):
        """Does nothing (ROS2 compatibility stand-in)."""

    def get_nodes(self):
        """Return nodes that have been added to this executor."""
        with self._nodes_lock:
            return list(self._nodes)

    def spin(self):
        """Waits until ROS shut down."""
        ros1.spin()

    def spin_until_future_complete(self, future, timeout_sec=None):
        """
        Waits until future complete or timeout reached or ROS shut down.

        @param   future       the future to wait on until done.
        @param   timeout_sec  maximum seconds to wait. Block forever if None or negative.
                              Don't wait if 0.
        """
        ros1.spin_until_future_complete(future, timeout_sec)

    def spin_once(self, timeout_sec=None):
        """
        Waits until timeout, or forever if timeout None or negative, or until ROS shutdown.

        @param   timeout_sec  seconds to wait. Block forever if None or negative. Don't wait if 0.
        """
        ros1.spin_once(timeout_sec)

    def spin_once_until_future_complete(self, future, timeout_sec=None):
        """
        Waits until future complete or timeout reached or ROS shut down.

        @param   future       the future to wait on until done.
        @param   timeout_sec  maximum seconds to wait. Block forever if None or negative.
                              Don't wait if 0.
        """
        ros1.spin_until_future_complete(future, timeout_sec)



class SingleThreadedExecutor(Executor):
    """API stand-in for ROS2 compatibility."""


class MultiThreadedExecutor(Executor):
    """API stand-in for ROS2 compatibility."""


__all__ = [
    "ConditionReachedException", "Executor", "ExternalShutdownException",
    "MultiThreadedExecutor", "ShutdownException", "SingleThreadedExecutor", "TimeoutException"
]
