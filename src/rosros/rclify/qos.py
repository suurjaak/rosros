"""
Partial stand-in for `rclpy.qos` in ROS1.

Partially modified copy from ROS2 `rclpy.qos`,
at https://github.com/ros2/rclpy (`rclpy/rclpy/qos.py`),
released under the Apache 2.0 License.

Includes port of preset QoS profiles from ROS2 Middleware Interface and ROS2 Actions for ROS1.

QoS profile names and values taken from:
- https://github.com/ros2/rmw (`include/rmw/qos_profiles.h`, `include/rmw/types.h`)
- https://github.com/ros2/rcl (`include/rcl_action/default_qos.h`)

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     16.02.2022
@modified    16.06.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rclify.qos

# Original file copyright notice:
#
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from enum import Enum
from enum import IntEnum

from rospy import Duration


class QoSPolicyKind(IntEnum):
    """Enum for types of QoS policies that a Publisher or Subscription can set."""

    INVALID     = 1 << 0  # 2 ** 0
    DURABILITY  = 1 << 1  # 2 ** 1
    DEADLINE    = 1 << 2  # 2 ** 2
    LIVELINESS  = 1 << 3  # 2 ** 3
    RELIABILITY = 1 << 4  # 2 ** 4
    HISTORY     = 1 << 5  # 2 ** 5
    LIFESPAN    = 1 << 6  # 2 ** 6


class QoSPolicyEnum(IntEnum):
    """
    Base for QoS Policy enumerations.

    Provides helper function to filter keys for utilities.
    """

    @classmethod
    def short_keys(cls):
        """Return a list of shortened typing-friendly enum values."""
        return [k.lower() for k in cls.__members__.keys() if not k.startswith('RMW')]

    @classmethod
    def get_from_short_key(cls, name):
        """Retrieve a policy type from a short name, case-insensitive."""
        return cls[name.upper()].value

    @property
    def short_key(self):
        for k, v in self.__class__.__members__.items():
            if k.startswith('RMW'):
                continue
            if self.value is v:
                return k.lower()
        raise AttributeError(
            'failed to find value %s in %s' %
            (self.value, self.__class__.__name__))


class HistoryPolicy(QoSPolicyEnum):
    """Enum for QoS History settings."""

    RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT = 0
    RMW_QOS_POLICY_HISTORY_KEEP_LAST      = 1
    RMW_QOS_POLICY_HISTORY_KEEP_ALL       = 2
    RMW_QOS_POLICY_HISTORY_UNKNOWN        = 3
    SYSTEM_DEFAULT = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT
    KEEP_LAST      = RMW_QOS_POLICY_HISTORY_KEEP_LAST
    KEEP_ALL       = RMW_QOS_POLICY_HISTORY_KEEP_ALL
    UNKNOWN        = RMW_QOS_POLICY_HISTORY_UNKNOWN


# Alias with the old name, for retrocompatibility
QoSHistoryPolicy = HistoryPolicy


class ReliabilityPolicy(QoSPolicyEnum):
    """
    Enum for QoS Reliability settings.

    This enum matches the one defined in rmw/types.h
    """

    RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT = 0
    RMW_QOS_POLICY_RELIABILITY_RELIABLE       = 1
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT    = 2
    RMW_QOS_POLICY_RELIABILITY_UNKNOWN        = 3
    SYSTEM_DEFAULT = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT
    RELIABLE       = RMW_QOS_POLICY_RELIABILITY_RELIABLE
    BEST_EFFORT    = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
    UNKNOWN        = RMW_QOS_POLICY_RELIABILITY_UNKNOWN


# Alias with the old name, for retrocompatibility
QoSReliabilityPolicy = ReliabilityPolicy


class DurabilityPolicy(QoSPolicyEnum):
    """
    Enum for QoS Durability settings.

    This enum matches the one defined in rmw/types.h
    """

    RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT  = 0
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL = 1
    RMW_QOS_POLICY_DURABILITY_VOLATILE        = 2
    RMW_QOS_POLICY_DURABILITY_UNKNOWN         = 3
    SYSTEM_DEFAULT  = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT
    TRANSIENT_LOCAL = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
    VOLATILE        = RMW_QOS_POLICY_DURABILITY_VOLATILE
    UNKNOWN         = RMW_QOS_POLICY_DURABILITY_UNKNOWN


# Alias with the old name, for retrocompatibility
QoSDurabilityPolicy = DurabilityPolicy


class LivelinessPolicy(QoSPolicyEnum):
    """
    Enum for QoS Liveliness settings.

    This enum matches the one defined in rmw/types.h
    """

    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT  = 0
    RMW_QOS_POLICY_LIVELINESS_AUTOMATIC       = 1
    RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC = 3
    RMW_QOS_POLICY_LIVELINESS_UNKNOWN         = 4
    SYSTEM_DEFAULT  = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT
    AUTOMATIC       = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
    MANUAL_BY_TOPIC = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC
    UNKNOWN         = RMW_QOS_POLICY_LIVELINESS_UNKNOWN


# Alias with the old name, for retrocompatibility
QoSLivelinessPolicy = LivelinessPolicy


DURATION_UNSPEFICIED              = Duration(0)
LIVELINESS_LEASE_DURATION_DEFAULT = DURATION_UNSPEFICIED
DEADLINE_DEFAULT                  = DURATION_UNSPEFICIED
LIFESPAN_DEFAULT                  = DURATION_UNSPEFICIED
DEPTH_SYSTEM_DEFAULT              = 0
LIVELINESS_UNKNOWN                = 4   # From ros2/rmw: include/rmw/types.h


DEFAULT = dict(
    history                         = HistoryPolicy.KEEP_LAST,
    depth                           = 10,
    reliability                     = ReliabilityPolicy.RELIABLE,
    durability                      = DurabilityPolicy.VOLATILE,
    deadline                        = DEADLINE_DEFAULT,
    lifespan                        = LIFESPAN_DEFAULT,
    liveliness                      = LivelinessPolicy.SYSTEM_DEFAULT,
    liveliness_lease_duration       = LIVELINESS_LEASE_DURATION_DEFAULT,
    avoid_ros_namespace_conventions = False,
)


UNKNOWN = dict(
    history                         = HistoryPolicy.UNKNOWN,
    depth                           = DEPTH_SYSTEM_DEFAULT,
    reliability                     = ReliabilityPolicy.UNKNOWN,
    durability                      = DurabilityPolicy.UNKNOWN,
    deadline                        = DEADLINE_DEFAULT,
    lifespan                        = LIFESPAN_DEFAULT,
    liveliness                      = LivelinessPolicy.SYSTEM_DEFAULT,
    liveliness_lease_duration       = LIVELINESS_LEASE_DURATION_DEFAULT,
    avoid_ros_namespace_conventions = False,
)


SYSTEM_DEFAULT = dict(
    history                         = HistoryPolicy.SYSTEM_DEFAULT,
    depth                           = DEPTH_SYSTEM_DEFAULT,
    reliability                     = ReliabilityPolicy.SYSTEM_DEFAULT,
    durability                      = DurabilityPolicy.SYSTEM_DEFAULT,
    deadline                        = DEADLINE_DEFAULT,
    lifespan                        = LIFESPAN_DEFAULT,
    liveliness                      = LivelinessPolicy.SYSTEM_DEFAULT,
    liveliness_lease_duration       = LIVELINESS_LEASE_DURATION_DEFAULT,
    avoid_ros_namespace_conventions = False,
)


SENSOR_DATA = dict(
    history                         = HistoryPolicy.KEEP_LAST,
    depth                           = 5,
    reliability                     = ReliabilityPolicy.BEST_EFFORT,
    durability                      = DurabilityPolicy.VOLATILE,
    deadline                        = DEADLINE_DEFAULT,
    lifespan                        = LIFESPAN_DEFAULT,
    liveliness                      = LivelinessPolicy.SYSTEM_DEFAULT,
    liveliness_lease_duration       = LIVELINESS_LEASE_DURATION_DEFAULT,
    avoid_ros_namespace_conventions = False,
)


SERVICES = dict(
    history                         = HistoryPolicy.KEEP_LAST,
    depth                           = 10,
    reliability                     = ReliabilityPolicy.RELIABLE,
    durability                      = DurabilityPolicy.VOLATILE,
    deadline                        = DEADLINE_DEFAULT,
    lifespan                        = LIFESPAN_DEFAULT,
    liveliness                      = LivelinessPolicy.SYSTEM_DEFAULT,
    liveliness_lease_duration       = LIVELINESS_LEASE_DURATION_DEFAULT,
    avoid_ros_namespace_conventions = False,
)


PARAMETERS = dict(
    history                         = HistoryPolicy.KEEP_LAST,
    depth                           = 1000,
    reliability                     = ReliabilityPolicy.RELIABLE,
    durability                      = DurabilityPolicy.VOLATILE,
    deadline                        = DEADLINE_DEFAULT,
    lifespan                        = LIFESPAN_DEFAULT,
    liveliness                      = LivelinessPolicy.SYSTEM_DEFAULT,
    liveliness_lease_duration       = LIVELINESS_LEASE_DURATION_DEFAULT,
    avoid_ros_namespace_conventions = False,
)


PARAMETER_EVENTS = dict(
    history                         = HistoryPolicy.KEEP_LAST,
    depth                           = 1000,
    reliability                     = ReliabilityPolicy.RELIABLE,
    durability                      = DurabilityPolicy.VOLATILE,
    deadline                        = DEADLINE_DEFAULT,
    lifespan                        = LIFESPAN_DEFAULT,
    liveliness                      = LivelinessPolicy.SYSTEM_DEFAULT,
    liveliness_lease_duration       = LIVELINESS_LEASE_DURATION_DEFAULT,
    avoid_ros_namespace_conventions = False,
)


ACTIONS = dict(
    history                         = HistoryPolicy.KEEP_LAST,
    depth                           = 1,
    reliability                     = ReliabilityPolicy.RELIABLE,
    durability                      = DurabilityPolicy.TRANSIENT_LOCAL,
    deadline                        = DEADLINE_DEFAULT,
    lifespan                        = LIFESPAN_DEFAULT,
    liveliness                      = LivelinessPolicy.SYSTEM_DEFAULT,
    liveliness_lease_duration       = LIVELINESS_LEASE_DURATION_DEFAULT,
    avoid_ros_namespace_conventions = False,
)


PresetNames = {
    "qos_profile_default":                   DEFAULT,
    "qos_profile_unknown":                   UNKNOWN,
    "qos_profile_system_default":            SYSTEM_DEFAULT,
    "qos_profile_sensor_data":               SENSOR_DATA,
    "qos_profile_services_default":          SERVICES,
    "qos_profile_parameters":                PARAMETERS,
    "qos_profile_parameter_events":          PARAMETER_EVENTS,
    "rcl_action_qos_profile_status_default": ACTIONS,
}


def qos_policy_name_from_kind(policy_kind):
    """Get QoS policy name from QoSPolicyKind enum."""
    if isinstance(policy_kind, QoSPolicyKind):
        return "%s_QOS_POLICY" % policy_kind.name  # "DEADLINE_QOS_POLICY" etc
    return "INVALID_QOS_POLICY"


class InvalidQoSProfileException(Exception):
    """Raised when constructing a QoSProfile with invalid arguments."""

    def __init__(self, *args):
        Exception.__init__(self, 'Invalid QoSProfile', *args)


class QoSProfile:
    """Define Quality of Service policies."""

    # default QoS profile not exposed to the user to encourage them to think about QoS settings
    __qos_profile_default_dict = PresetNames['qos_profile_default']

    __slots__ = [
        '_history',
        '_depth',
        '_reliability',
        '_durability',
        '_lifespan',
        '_deadline',
        '_liveliness',
        '_liveliness_lease_duration',
        '_avoid_ros_namespace_conventions',
    ]

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %r' % kwargs.keys()

        if 'history' not in kwargs:
            if 'depth' not in kwargs:
                raise InvalidQoSProfileException('History and/or depth settings are required.')
            kwargs['history'] = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST

        self.history = kwargs.get('history')

        if (
            QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST == self.history and
            'depth' not in kwargs
        ):
            raise InvalidQoSProfileException('History set to KEEP_LAST without a depth setting.')

        self.depth = kwargs.get('depth', QoSProfile.__qos_profile_default_dict['depth'])
        self.reliability = kwargs.get(
            'reliability', QoSProfile.__qos_profile_default_dict['reliability'])
        self.durability = kwargs.get(
            'durability', QoSProfile.__qos_profile_default_dict['durability'])
        self.lifespan = kwargs.get('lifespan', QoSProfile.__qos_profile_default_dict['lifespan'])
        self.deadline = kwargs.get('deadline', QoSProfile.__qos_profile_default_dict['deadline'])
        self.liveliness = kwargs.get(
            'liveliness', QoSProfile.__qos_profile_default_dict['liveliness'])
        self.liveliness_lease_duration = kwargs.get(
            'liveliness_lease_duration',
            QoSProfile.__qos_profile_default_dict['liveliness_lease_duration'])
        self.avoid_ros_namespace_conventions = kwargs.get(
            'avoid_ros_namespace_conventions',
            QoSProfile.__qos_profile_default_dict['avoid_ros_namespace_conventions'])

    @property
    def history(self):
        """
        Get field 'history'.

        :returns: history attribute
        :rtype: QoSHistoryPolicy
        """
        return self._history

    @history.setter
    def history(self, value):
        assert isinstance(value, (QoSHistoryPolicy, int))
        self._history = QoSHistoryPolicy(value)

    @property
    def reliability(self):
        """
        Get field 'reliability'.

        :returns: reliability attribute
        :rtype: QoSReliabilityPolicy
        """
        return self._reliability

    @reliability.setter
    def reliability(self, value):
        assert isinstance(value, (QoSReliabilityPolicy, int))
        self._reliability = QoSReliabilityPolicy(value)

    @property
    def durability(self):
        """
        Get field 'durability'.

        :returns: durability attribute
        :rtype: QoSDurabilityPolicy
        """
        return self._durability

    @durability.setter
    def durability(self, value):
        assert isinstance(value, (QoSDurabilityPolicy, int))
        self._durability = QoSDurabilityPolicy(value)

    @property
    def depth(self):
        """
        Get field 'depth'.

        :returns: depth attribute
        :rtype: int
        """
        return self._depth

    @depth.setter
    def depth(self, value):
        assert isinstance(value, int)
        self._depth = value

    @property
    def lifespan(self):
        """
        Get field 'lifespan'.

        :returns: lifespan attribute
        :rtype: Duration
        """
        return self._lifespan

    @lifespan.setter
    def lifespan(self, value):
        assert isinstance(value, Duration)
        self._lifespan = value

    @property
    def deadline(self):
        """
        Get field 'deadline'.

        :returns: deadline attribute.
        :rtype: Duration
        """
        return self._deadline

    @deadline.setter
    def deadline(self, value):
        assert isinstance(value, Duration)
        self._deadline = value

    @property
    def liveliness(self):
        """
        Get field 'liveliness'.

        :returns: liveliness attribute
        :rtype: QoSLivelinessPolicy
        """
        return self._liveliness

    @liveliness.setter
    def liveliness(self, value):
        assert isinstance(value, (QoSLivelinessPolicy, int))
        self._liveliness = QoSLivelinessPolicy(value)

    @property
    def liveliness_lease_duration(self):
        """
        Get field 'liveliness_lease_duration'.

        :returns: liveliness_lease_duration attribute.
        :rtype: Duration
        """
        return self._liveliness_lease_duration

    @liveliness_lease_duration.setter
    def liveliness_lease_duration(self, value):
        assert isinstance(value, Duration)
        self._liveliness_lease_duration = value

    @property
    def avoid_ros_namespace_conventions(self):
        """
        Get field 'avoid_ros_namespace_conventions'.

        :returns: avoid_ros_namespace_conventions attribute
        :rtype: bool
        """
        return self._avoid_ros_namespace_conventions

    @avoid_ros_namespace_conventions.setter
    def avoid_ros_namespace_conventions(self, value):
        assert isinstance(value, bool)
        self._avoid_ros_namespace_conventions = value

    def get_c_qos_profile(self):
        """Returns None (ROS2 API compatibility stand-in)."""
        return None

    def __eq__(self, other):
        if not isinstance(other, QoSProfile):
            return False
        return all(
            self.__getattribute__(slot) == other.__getattribute__(slot)
            for slot in self.__slots__)


# The details of the following profiles can be found at
# 1. ROS QoS principles:
#    https://design.ros2.org/articles/qos.html
# 2. ros2/rmw : rmw/include/rmw/qos_profiles.h

#: Used for initialization. Should not be used as the actual QoS profile.
qos_profile_unknown = QoSProfile(**PresetNames['qos_profile_unknown'])
#: Uses the default QoS settings defined in the DDS vendor tool
qos_profile_system_default = QoSProfile(**PresetNames['qos_profile_system_default'])
#: For sensor data, using best effort reliability and small queue depth
qos_profile_sensor_data = QoSProfile(**PresetNames['qos_profile_sensor_data'])
#: For services, using reliable reliability and volatile durability
qos_profile_services_default = QoSProfile(**PresetNames['qos_profile_services_default'])
#: For parameter communication. Similar to service QoS profile but with larger
#: queue depth so that requests do not get lost.
qos_profile_parameters = QoSProfile(**PresetNames['qos_profile_parameters'])
#: For parameter change events. Currently same as the QoS profile for
#: parameters.
qos_profile_parameter_events = QoSProfile(**PresetNames['qos_profile_parameter_events'])

# Separate rcl_action profile defined at
# ros2/rcl : rcl/rcl_action/include/rcl_action/default_qos.h
#
#: For actions, using reliable reliability, transient-local durability.
qos_profile_action_status_default = QoSProfile(**PresetNames['rcl_action_qos_profile_status_default'])


class QoSPresetProfiles(Enum):
    UNKNOWN = qos_profile_unknown
    SYSTEM_DEFAULT = qos_profile_system_default
    SENSOR_DATA = qos_profile_sensor_data
    SERVICES_DEFAULT = qos_profile_services_default
    PARAMETERS = qos_profile_parameters
    PARAMETER_EVENTS = qos_profile_parameter_events
    ACTION_STATUS_DEFAULT = qos_profile_action_status_default

    """Noted that the following are duplicated from QoSPolicyEnum.

    Our supported version of Python3 (3.5) doesn't have a fix that allows mixins on Enum.
    """
    @classmethod
    def short_keys(cls):
        """Return a list of shortened typing-friendly enum values."""
        return [k.lower() for k in cls.__members__.keys() if not k.startswith('RMW')]

    @classmethod
    def get_from_short_key(cls, name):
        """Retrieve a policy type from a short name, case-insensitive."""
        return cls[name.upper()].value
