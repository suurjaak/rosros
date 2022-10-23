"""
Stand-ins for `rospy.msg` in ROS2.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     30.05.2022
@modified    23.10.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rospify.msg
import std_msgs.msg

from . import exceptions
from . topics import Message


Header = std_msgs.msg.Header


class AnyMsg(Message):
    """
    Stand-in for `rospy.AnyMsg`.

    Message class to use for subscribing to any topic regardless of type.
    Incoming messages are not deserialized. Instead, the raw serialized data
    can be accessed via the buff property.

    Caveat in ROS2: subscription will be made with the first available published type
    on this topic, creation will fail if no publisher is available. Will not provide
    messages of different types.
    """
    _md5sum     = "*"
    _type       = "*"
    _has_header = False
    _full_text  = ""
    __slots__   = ["_buff"]

    def __init__(self, *args):
        """
        Constructor. Does not accept any arguments.
        """
        if len(args) != 0:
            raise exceptions.ROSException("AnyMsg does not accept arguments")
        self._buff = None

    def serialize(self, buff):
        """AnyMsg provides an implementation so that a node can forward messages w/o (de)serialization."""
        if self._buff is None:
            raise exceptions.ROSException("AnyMsg is not initialized")
        buff.write(self._buff)

    def deserialize(self, str):
        """Copies raw buffer into self._buff, returns self."""
        self._buff = str
        return self

    @classmethod
    def __subclasshook__(cls, C):
        """Returns true if C is AnyMsg or descendant, else `NotImplemented`."""
        if getattr(C, "_type", None) == cls._type: return True
        return NotImplemented


__all__ = [
    "AnyMsg", "Header"
]
