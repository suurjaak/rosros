"""
Partial stand-in of ROS2 `rclpy.parameter`.

And some parameter-related classes from `rcl_interfaces.msg` for ROS1.

Heavily modified copy from ROS2 `rclpy.parameter`,
at https://github.com/ros2/rclpy (`rclpy/rclpy/parameter.py`),
released under the Apache 2.0 License.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     17.02.2022
@modified    24.06.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rclify.parameter

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
import array
from enum import Enum

PARAMETER_SEPARATOR_STRING = "."


class Parameter:
    """ROS node parameter, a stand-in for ROS2 `rclpy.parameter.Parameter` in ROS1."""

    class Type(Enum):
        """ROS parameter type, a stand-in for ROS2 `rclpy.parameter.Parameter.Type` in ROS1."""

        NOT_SET       = 0
        BOOL          = 1
        INTEGER       = 2
        DOUBLE        = 3
        STRING        = 4
        BYTE_ARRAY    = 5
        BOOL_ARRAY    = 6
        INTEGER_ARRAY = 7
        DOUBLE_ARRAY  = 8
        STRING_ARRAY  = 9

        @classmethod
        def from_parameter_value(cls, parameter_value):
            """
            Returns `Parameter.Type` for given value.

            @return   a `Parameter.Type` corresponding to the instance type of the given value
            @throws   `TypeError` if the conversion to a type was not possible
            """
            if isinstance(parameter_value, (list, tuple, array.array)):
                for enumtype, valtype in Parameter.ARRAYS.items():
                    if all(isinstance(v, valtype) for v in parameter_value):
                        return enumtype
                raise TypeError("The given value is not a list of one of the allowed types.")
            for enumtype, valtype in Parameter.SCALARS.items():
                if isinstance(parameter_value, valtype):
                    return enumtype
            raise TypeError("The given value is not one of the allowed types.")

        def check(self, parameter_value):
            """Returns whether given value matches this type."""
            if self in Parameter.SCALARS:
                return isinstance(parameter_value, Parameter.SCALARS[self])
            if self not in Parameter.ARRAYS \
            or not isinstance(parameter_value, (list, tuple, array.array)):
                return False
            if self is self.BYTE_ARRAY:
                return all(isinstance(v, bytes) and len(v) == 1 for v in parameter_value)
            return all(isinstance(v, Parameter.ARRAYS[self]) for v in parameter_value)


    ## Mapping from scalar types to Python types
    SCALARS = {Type.NOT_SET: type(None), Type.BOOL:   bool, Type.INTEGER: int,
               Type.DOUBLE:  float,      Type.STRING: str}

    ## Mapping from array types to array element Python types
    ARRAYS = {Type.BYTE_ARRAY:   bytes, Type.BOOL_ARRAY:   bool, Type.INTEGER_ARRAY: int,
              Type.DOUBLE_ARRAY: float, Type.STRING_ARRAY: str}


    def __init__(self, name, type_=None, value=None):
        """Raises error if unknown type or given value not of given type."""
        if type_ is None:
            type_ = Parameter.Type.from_parameter_value(value)  # Raises if unknown type
        if not isinstance(type_, Parameter.Type):
            raise TypeError("Type must be an instance of %r" % Parameter.Type)
        if not type_.check(value):
            raise ValueError("Type '%s' and value '%s' do not agree" % (type_, value))

        self._type_ = type_
        self._name  = name
        self._value = value

    @property
    def name(self):
        return self._name

    @property
    def type_(self):
        return self._type_

    @property
    def value(self):
        return self._value

    def get_parameter_value(self):
        """Returns `ParameterValue`."""
        parameter_value = ParameterValue(type=self._type_.value)
        if Parameter.Type.BOOL == self._type_:
            parameter_value.bool_value = self.value
        elif Parameter.Type.INTEGER == self._type_:
            parameter_value.integer_value = self.value
        elif Parameter.Type.DOUBLE == self._type_:
            parameter_value.double_value = self.value
        elif Parameter.Type.STRING == self._type_:
            parameter_value.string_value = self.value
        elif Parameter.Type.BYTE_ARRAY == self._type_:
            parameter_value.byte_array_value = self.value
        elif Parameter.Type.BOOL_ARRAY == self._type_:
            parameter_value.bool_array_value = self.value
        elif Parameter.Type.INTEGER_ARRAY == self._type_:
            parameter_value.integer_array_value = self.value
        elif Parameter.Type.DOUBLE_ARRAY == self._type_:
            parameter_value.double_array_value = self.value
        elif Parameter.Type.STRING_ARRAY == self._type_:
            parameter_value.string_array_value = self.value
        return parameter_value




class ParameterValue:
    """
    Used to determine which of the next *_value fields are set.
    `Parameter.Type.NOT_SET` indicates that the parameter was not set
    (if gotten) or is uninitialized.

    Stand-in for ROS2 `rcl_interfaces.msg.ParameterValue` in ROS1.
    """
    def __init__(self, type=0, bool_value=False, integer_value=0, double_value=0.0, string_value="",
                 byte_array_value=None, bool_array_value=None, integer_array_value=None,
                 double_array_value=None, string_array_value=None):
        ## The type of this parameter, which corresponds to the appropriate field below.
        self.type                = type
        ## Boolean value, can be either true or false.
        self.bool_value          = bool_value
        ## Integer value.
        self.integer_value       = integer_value
        ## A double precision floating point value.
        self.double_value        = double_value
        ## A textual value with no practical length limit
        self.string_value        = string_value
        ## An array of bytes, used for non-textual information.
        self.byte_array_value    = byte_array_value or []
        ## An array of boolean values.
        self.bool_array_value    = bool_array_value or []
        ## An array of 64-bit integer values.
        self.integer_array_value = integer_array_value or []
        ## An array of 64-bit floating point values.
        self.double_array_value  = double_array_value or []
        ## An array of string values.
        self.string_array_value  = string_array_value or []

    def get_value(self):
        """Returns raw value according to type."""
        value = None
        type_ = Parameter.Type(self.type)
        if Parameter.Type.BOOL == type_:
            value = self.bool_value
        elif Parameter.Type.INTEGER == type_:
            value = self.integer_value
        elif Parameter.Type.DOUBLE == type_:
            value = self.double_value
        elif Parameter.Type.STRING == type_:
            value = self.string_value
        elif Parameter.Type.BYTE_ARRAY == type_:
            value = self.byte_array_value
        elif Parameter.Type.BOOL_ARRAY == type_:
            value = self.bool_array_value
        elif Parameter.Type.INTEGER_ARRAY == type_:
            value = self.integer_array_value
        elif Parameter.Type.DOUBLE_ARRAY == type_:
            value = self.double_array_value
        elif Parameter.Type.STRING_ARRAY == type_:
            value = self.string_array_value
        return value


class ParameterDescriptor:
    """
    This is the message to communicate a parameter's descriptor.

    Stand-in for ROS2 `rcl_interfaces.msg.ParameterDescriptor` in ROS1.
    """

    def __init__(self, name="", type=0, description="", additional_constraints="", read_only=False,
                 dynamic_typing=False, floating_point_range=None, integer_range=None):
        ## The name of the parameter.
        self.name                   = name or ""
        ## Enum values are defined in `Parameter.Type`.
        self.type                   = type or 0
        ## Description of the parameter, visible from introspection tools.
        self.description            = description or ""
        ## Plain English description of additional constraints which cannot be expressed
        ## with the available constraints, e.g. "only prime numbers".
        ## By convention, this should only be used to clarify constraints which cannot
        ## be completely expressed with the parameter constraints below.
        self.additional_constraints = additional_constraints or ""
        ## If 'true' then the value cannot change after it has been initialized.
        self.read_only              = read_only or False
        ## If true, the parameter is allowed to change type.
        self.dynamic_typing         = dynamic_typing or False
        ## FloatingPointRange consists of a from_value, a to_value, and a step.
        ## Mutually exclusive with `integer_range`.
        self.floating_point_range   = floating_point_range or []
        ## IntegerRange consists of a from_value, a to_value, and a step.
        ## Mutually exclusive with `floating_point_range`.
        self.integer_range          = integer_range or []


class SetParametersResult:
    """
    This is the message to communicate the result of setting parameters.

    Stand-in for ROS2 `rcl_interfaces.msg.SetParametersResult` in ROS1.
    """

    def __init__(self, successful=False, reason=""):
        ## A true value of the same index indicates that the parameter was set
        ## successfully. A false value indicates the change was rejected.
        self.successful = successful or False
        ## Reason why the setting was either successful or a failure. This should only be
        ## used for logging and user interfaces.
        self.reason     = reason or ""


class FloatingPointRange:
    """
    Represents bounds and a step value for a floating point typed parameter.

    Stand-in for ROS2 `rcl_interfaces.msg.FloatingPointRange` in ROS1.
    """
    def __init__(self, from_value=0.0, to_value=0.0, step=0.0):
        ## Start value for valid values, inclusive.
        self.from_value = from_value or 0.0
        ## End value for valid values, inclusive.
        self.to_value   = to_value   or 0.0
        ## Size of valid steps between the from and to bound.
        ## Step is considered to be a magnitude, therefore negative values are treated
        ## the same as positive values, and a step value of zero implies a continuous
        ## range of values.
        ##
        ## Ideally, the step would be less than or equal to the distance between the
        ## bounds, as well as an even multiple of the distance between the bounds, but
        ## neither are required.
        ##
        ## If the absolute value of the step is larger than or equal to the distance
        ## between the two bounds, then the bounds will be the only valid values. e.g. if
        ## the range is defined as {from_value: 1.0, to_value: 2.0, step: 5.0} then the
        ## valid values will be 1.0 and 2.0.
        ##
        ## If the step is less than the distance between the bounds, but the distance is
        ## not a multiple of the step, then the "to" bound will always be a valid value,
        ## e.g. if the range is defined as {from_value: 2.0, to_value: 5.0, step: 2.0}
        ## then the valid values will be 2.0, 4.0, and 5.0.
        self.step       = step       or 0.0


class IntegerRange:
    """
    Represents bounds and a step value for an integer typed parameter.

    Stand-in for ROS2 `rcl_interfaces.msg.IntegerRange` in ROS1.
    """
    def __init__(self, from_value=0, to_value=0, step=0):
        ## Start value for valid values, inclusive.
        self.from_value = from_value or 0
        ## End value for valid values, inclusive.
        self.to_value   = to_value   or 0
        ## Size of valid steps between the from and to bound.
        ##
        ## A step value of zero implies a continuous range of values. Ideally, the step
        ## would be less than or equal to the distance between the bounds, as well as an
        ## even multiple of the distance between the bounds, but neither are required.
        ##
        ## If the absolute value of the step is larger than or equal to the distance
        ## between the two bounds, then the bounds will be the only valid values. e.g. if
        ## the range is defined as {from_value: 1, to_value: 2, step: 5} then the valid
        ## values will be 1 and 2.
        ##
        ## If the step is less than the distance between the bounds, but the distance is
        ## not a multiple of the step, then the "to" bound will always be a valid value,
        ## e.g. if the range is defined as {from_value: 2, to_value: 5, step: 2} then
        ## the valid values will be 2, 4, and 5.
        self.step       = step       or 0


__all__ = [
    "PARAMETER_SEPARATOR_STRING", "Parameter", "ParameterValue", "ParameterDescriptor",
    "SetParametersResult", "FloatingPointRange", "IntegerRange",
]
