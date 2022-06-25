"""
Stand-ins for `rospy.topics` in ROS2.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     30.05.2022
@modified    25.06.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rospify.topics
import abc

import rclpy.publisher
import rclpy.subscription

from .. import ros2
from . import exceptions
#from . msg import AnyMsg  # Imported late to avoid circular import


class Message(abc.ABC):
    """Abstract base class of Message data classes auto-generated from msg files."""

    @classmethod
    def __subclasshook__(cls, C):
        """Returns true if C is a ROS2 message class, else `NotImplemented`."""
        return True if ros2.is_ros_message(C) else NotImplemented


## Empty stand-in for `rospy.SubscribeListener`
class SubscribeListener: pass


class Publisher(abc.ABC):
    """Stand-in for `rospy.Publisher`, wrapping the creation of ROS2 publisher."""

    def __new__(cls, name, data_class, subscriber_listener=None, tcp_nodelay=None,
                latch=False, headers=None, queue_size=None):
        """
        Constructor.

        @param   name                 resource name of topic, e.g. 'laser'.
        @param   data_class           message class for serialization
        @param   subscriber_listener  ignored (ROS1 compatibility stand-in)
        @param   tcp_nodelay          ignored (ROS1 compatibility stand-in)
        @param   latch                if True, the last message published is 'latched',
                                      meaning that any compatible subscribers will be sent
                                      that message immediately upon connection
        @param   headers              ignored (ROS1 compatibility stand-in)
        @param   queue_size           the queue size used for asynchronously publishing messages
                                      from different threads. A size of zero means an infinite queue,
                                      which can be dangerous.  When the keyword is not being used
                                      or when None is passed, all publishing will happen synchronously
                                      and a warning message will be printed.

        @throws  ROSException         if parameters are invalid
        """
        try:
            return ros2.create_publisher(name, data_class, latch=latch, queue_size=queue_size)
        except Exception as e:
            raise exceptions.ROSException() from e

    @classmethod
    def __subclasshook__(cls, C):
        """Returns true if C is a ROS2 publisher class, else `NotImplemented`."""
        return True if issubclass(C, rclpy.publisher.Publisher) else NotImplemented


class Subscriber(abc.ABC):
    """Stand-in for `rospy.Subscriber`, wrapping the creation of ROS2 subscription."""

    def __new__(cls, name, data_class, callback=None, callback_args=None,
                queue_size=None, buff_size=None, tcp_nodelay=None):
        """
        Returns rclpy.subscription.Subscription instance, supporting `AnyMsg`.

        @param   name           graph resource name of topic, e.g. 'laser'
        @param   data_class     data type class to use for messages, e.g. std_msgs.msg.String
        @param   callback       function to call ( fn(data)) when data is received.
                                If callback_args is set, the function must accept the callback_args
                                as a second argument, i.e. fn(data, callback_args).
                                NOTE: Additional callbacks can be added using add_callback().
        @param   callback_args  additional arguments to pass to the callback.
                                This is useful when you wish to reuse the same callback
                                for multiple subscriptions.
        @param   queue_size     maximum number of messages to receive at a time.
                                This will generally be 1 or None (infinite, default).
        @param   buff_size      ignored (ROS1 compatibility stand-ine)
        @param   tcp_nodelay    ignored (ROS1 compatibility stand-ine)

        @throws  ROSException   if parameters are invalid
        """
        from . msg import AnyMsg  # Imported late to avoid circular import
        try:
            raw = False
            if issubclass(data_class, AnyMsg):  # Look up real type from ROS2 network
                infos = ros2.NODE.get_publishers_info_by_topic(name)
                infos = infos or ros2.NODE.get_subscriptions_info_by_topic(name)
                typename = next(ros2.canonical(x.topic_type) for x in infos)
                data_class, raw = ros2.get_message_class(typename), True
            sub = ros2.create_subscriber(name, data_class, callback, callback_args=callback_args,
                                         queue_size=queue_size or 0, raw=raw)
            sub._anymsg = raw
        except Exception as e:
            raise exceptions.ROSException() from e
        return sub

    @classmethod
    def __subclasshook__(cls, C):
        """Returns true if C is a ROS2 subscription class, else `NotImplemented`."""
        return True if issubclass(C, rclpy.subscription.Subscription) else NotImplemented


__all__ = [
    "Message", "Publisher", "SubscribeListener", "Subscriber"
]
