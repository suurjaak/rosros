"""
Partial stand-in of ROS2 `rclpy.node` for ROS1.

Heavily modified copy from ROS2 `rclpy.node`,
at https://github.com/ros2/rclpy (`rclpy/rclpy/node.py`),
released under the Apache 2.0 License.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     14.02.2022
@modified    25.06.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rclify.node

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
import math
import os
import sys
import warnings

import rospy

from .. import api
from .. import ros1
from .. import util
from . import exceptions
from . clock import Clock, ClockType
from . parameter import PARAMETER_SEPARATOR_STRING, FloatingPointRange, IntegerRange, \
                        Parameter, ParameterDescriptor, ParameterValue, SetParametersResult
from . qos import DurabilityPolicy, QoSPresetProfiles
from . topic_endpoint_info import TopicEndpointInfo, TopicEndpointTypeEnum



class NodeNameNonExistentError(Exception):
    """Thrown when a node name is not found."""


class Node:
    """
    ROS2-like compatitibility class for ROS1.

    Declares the full interface of `rclpy.node.Node`, with partially implemented
    functionality.
    """

    ## Relative tolerance for floating-point parameter range steps
    PARAM_REL_TOL = 1e-6


    def __init__(self, node_name, *, context=None, cli_args=None, namespace=None,
                 use_global_arguments=True, enable_rosout=True, start_parameter_services=None,
                 parameter_overrides=None, allow_undeclared_parameters=False,
                 automatically_declare_parameters_from_overrides=False):
        """
        Creates a ROS1 node.

        @param   node_name   a name to give to this node
        @param   context     ignored (ROS2 API compatibility stand-in)
        @param   cli_args    a list of strings of command line args to be used only by this node;
                             these arguments are used to extract remappings used by the node and other ROS specific
                             settings, as well as user defined non-ROS arguments
        @param   namespace   the namespace to which relative topic and service names will be prefixed

        @param   use_global_arguments      `False` if the node should ignore process-wide command line args
        @param   enable_rosout             `False` if the node should ignore rosout logging
        @param   start_parameter_services  ignored (ROS2 API compatibility stand-in)
        @param   parameter_overrides       list of `Parameter` overrides for initial values
                                           of parameters declared on the node

        @param   allow_undeclared_parameters                      true if undeclared parameters are allowed;
                                                                  this flag affects the behavior of parameter-related operations
        @param   automatically_declare_parameters_from_overrides  if True, `parameter_overrides`
                                                                  will be used to implicitly declare
                                                                  parameters on the node
        """
        if ros1.MASTER:
            raise Exception("ROS1 node already created, cannot create another")

        self.__publishers          = []
        self.__subscriptions       = []
        self.__clients             = []
        self.__services            = []
        self.__timers              = []
        self.__executor            = None
        self._parameters           = {}
        self._parameters_callbacks = []
        self._parameter_overrides  = {}
        self._descriptors          = {}
        self._allow_undeclared_parameters = allow_undeclared_parameters

        if namespace and namespace != "/":
            os.environ["ROS_NAMESPACE"] = namespace
            rospy.names._set_caller_id(util.namejoin(namespace, node_name))

        node_args = (sys.argv if use_global_arguments else []) + list(cli_args or [])
        ros1.init_node(node_name, node_args, enable_rosout=enable_rosout)

        self._parameter_overrides = {k: Parameter(k, value=v)
                                     for k, v in ros1.get_params(nested=False)}
        self._parameter_overrides.update({p.name: p for p in parameter_overrides or {}})
        if automatically_declare_parameters_from_overrides:
            self._parameters.update(self._parameter_overrides)
            self._descriptors.update({p: ParameterDescriptor() for p in self._parameters})
            for p in parameter_overrides or {}:
                ros1.set_param(p.name, p.value)

        self._clock = Clock(clock_type=ClockType.ROS_TIME)

    @property
    def publishers(self):
        """Yields publishers that have been created on this node."""
        yield from self.__publishers

    @property
    def subscriptions(self):
        """Yields subscriptions that have been created on this node."""
        yield from self.__subscriptions

    @property
    def clients(self):
        """Yields clients that have been created on this node."""
        yield from self.__clients

    @property
    def services(self):
        """Yields services that have been created on this node."""
        yield from self.__services

    @property
    def timers(self):
        """Yields timers that have been created on this node."""
        yield from self.__timers

    @property
    def guards(self):
        """Yields (ROS2 API compatibility stand-in)."""
        yield from []

    @property
    def waitables(self):
        """Yields (ROS2 API compatibility stand-in)."""
        yield from []

    @property
    def executor(self):
        """Returns None (ROS2 API compatibility stand-in)."""
        return self.__executor

    @executor.setter
    def executor(self, new_executor):
        """Does nothing (ROS2 API compatibility stand-in)."""
        self.__executor = new_executor

    @property
    def context(self):
        """Returns None (ROS2 API compatibility stand-in)."""
        return None

    @property
    def default_callback_group(self):
        """Returns None (ROS2 API compatibility stand-in)."""
        return None

    @property
    def handle(self):
        """Returns None (ROS2 API compatibility stand-in)."""
        return None

    @handle.setter
    def handle(self, value):
        raise AttributeError("handle cannot be modified after node creation")

    def get_name(self):
        """Returns the name of the node, without namespace."""
        return rospy.get_name().rsplit("/", 1)[-1]

    def get_namespace(self):
        """Returns the namespace of the node."""
        return "/" + rospy.get_namespace().strip("/")

    def get_clock(self):
        """Returns `clock.Clock` using ROS time."""
        return self._clock

    def get_logger(self):
        """Get the node's logger."""
        return ros1.get_logger()

    def declare_parameter(self, name, value=None, descriptor=None, ignore_override=False):
        """
        Sets and initializes a node parameter.

        This method, if successful, will result in any callback registered with
        `add_on_set_parameters_callback` to be called.

        @param   name             fully-qualified name of the parameter, including its namespace
        @param   value            value of the parameter to set, if any
        @param   descriptor       `ParameterDescriptor` instance, if any
        @param   ignore_override  whether to set value given here, ignoring any overrides
        @return                   `Parameter` instance with the final assigned value

        @throws  ParameterAlreadyDeclaredException  if the parameter has already been set
        @throws  InvalidParameterException          if the parameter name is invalid
        @throws  InvalidParameterValueException     if a registered callback rejects the parameter
        """
        descriptor = descriptor or ParameterDescriptor()
        return self.declare_parameters("", [(name, value, descriptor)], ignore_override)[0]

    def declare_parameters(self, namespace, parameters, ignore_override=False):
        """
        Sets a list of parameters.

        The tuples in the given parameter list shall contain the name for each parameter,
        optionally providing a value and a descriptor.
        For each entry in the list, a parameter with a name of "namespace.name"
        will be set.
        The resulting value for each set parameter will be returned, considering
        parameter overrides set upon node creation as the first choice,
        or provided parameter values as the second one.

        The name expansion is naive, so if you set the namespace to be "foo.",
        then the resulting parameter names will be like "foo..name".
        However, if the namespace is an empty string, then no leading '.' will be
        placed before each name, which would have been the case when naively
        expanding "namespace.name".
        This allows you to declare several parameters at once without a namespace.

        This method, if successful, will result in any callback registered with
        `add_on_set_parameters_callback` to be called once for each parameter.
        If one of those calls fail, an exception will be raised and the remaining parameters will
        not be declared.
        Parameters declared up to that point will not be undeclared.

        @param   namespace        namespace for parameters (empty string for node root namespace)
        @param   parameters       list of tuples with parameters to set,
                                  as (name, ) or (name, value) or (name, value, descriptor)
        @param   ignore_override  whether to set value given here, ignoring any overrides
        @return                   `Parameter` list with the final assigned values

        @throws  ParameterAlreadyDeclaredException  if a parameter has already been set
        @throws  InvalidParameterException          if a parameter name is invalid.
        @throws  InvalidParameterValueException     if a registered callback rejects any parameter
        @throws  TypeError                          if any tuple in `parameters`
                                                    does not match the annotated type
        """
        parameter_list = []
        descriptors = {}
        for index, parameter_tuple in enumerate(parameters):
            if len(parameter_tuple) < 1 or len(parameter_tuple) > 3:
                raise TypeError(
                    'Invalid parameter tuple length at index {index} in parameters list: '
                    '{parameter_tuple}; expecting length between 1 and 3'.format_map(locals())
                )

            value = None
            descriptor = ParameterDescriptor()

            # Get the values from the tuple, checking its types.
            # Use defaults if the tuple doesn't contain value and / or descriptor.
            try:
                name = parameter_tuple[0]
                assert \
                    isinstance(name, str), \
                    (
                        'First element {name} at index {index} in parameters list '
                        'is not a str.'.format_map(locals())
                    )

                # Get value from parameter overrides, of from tuple if it doesn't exist.
                if not ignore_override and name in self._parameter_overrides:
                    value = self._parameter_overrides[name].value
                else:
                    # This raises IndexError if value not given
                    value = parameter_tuple[1]

                # Get descriptor from tuple.
                descriptor = parameter_tuple[2]
                assert \
                    isinstance(descriptor, ParameterDescriptor), \
                    (
                        'Third element {descriptor} at index {index} in parameters list '
                        'is not a ParameterDescriptor.'.format_map(locals())
                    )
            except AssertionError as assertion_error:
                raise TypeError(assertion_error)
            except IndexError:
                # This means either value or descriptor were not defined which is fine.
                pass

            if namespace:
                name = "%s.%s" % (namespace, name)

            parameter_list.append(Parameter(name, value=value))
            descriptors.update({name: descriptor})

        parameters_already_declared = [
            parameter.name for parameter in parameter_list if parameter.name in self._parameters
        ]
        if any(parameters_already_declared):
            raise exceptions.ParameterAlreadyDeclaredException(parameters_already_declared)

        # Call the callback once for each of the parameters, using method that doesn't
        # check whether the parameter was declared beforehand or not.
        self._set_parameters(parameter_list, descriptors, raise_on_failure=True,
                             allow_undeclared_parameters=True)
        return self.get_parameters([parameter.name for parameter in parameter_list])

    def undeclare_parameter(self, name):
        """
        Unset a previously set parameter.

        This method will not cause a callback registered with
        `add_on_set_parameters_callback` to be called.

        @param   name  fully-qualified name of the parameter, including its namespace

        @throws  ParameterNotDeclaredException  if the parameter does not exist
        @throws  ParameterImmutableException    if the parameter was created as read-only
        """
        if self.has_parameter(name):
            if self._descriptors[name].read_only:
                raise exceptions.ParameterImmutableException(name)
            else:
                del self._parameters[name]
                del self._descriptors[name]
                ros1.delete_param(name)
        else:
            raise exceptions.ParameterNotDeclaredException(name)

    def has_parameter(self, name):
        """
        Returns whether parameter exists.

        @param   name  fully-qualified name of the parameter, including its namespace
        """
        return name in self._parameters

    def get_parameters(self, names):
        """
        Returns a list of parameters.

        @param   names  fully-qualified names of the parameters to get, including namespaces
        @return         The values for the given parameter names.
                        A default `Parameter` will be returned for undeclared parameters
                        if undeclared parameters are allowed.

        @throws  ParameterNotDeclaredException  if undeclared parameters are not allowed,
                                                and any given parameter is unknown
        """
        if not all(isinstance(name, str) for name in names):
            raise TypeError('All names must be instances of type str')
        return [self.get_parameter(name) for name in names]

    def get_parameter(self, name):
        """
        Returns a parameter by name.

        @param   name  fully-qualified name of the parameter, including its namespace
        @return        `Parameter` for the given name.
                       A default Parameter will be returned for an undeclared parameter
                       if undeclared parameters are allowed.

        @throws  ParameterNotDeclaredException  if undeclared parameters are not allowed,
                                                and the parameter is unknown
        """
        if self.has_parameter(name):
            return self._parameters[name]
        elif self._allow_undeclared_parameters:
            return Parameter(name, Parameter.Type.NOT_SET, None)
        else:
            raise exceptions.ParameterNotDeclaredException(name)

    def get_parameter_or(self, name, alternative_value=None):
        """
        Returns a parameter or the alternative value.

        If the alternative value is None, a default `Parameter` with the given name
        and `NOT_SET` type will be returned if the parameter was not declared.

        @param   name               fully-qualified name of the parameter, including its namespace
        @param   alternative_value  alternative parameter to return if unknown
        @return                     requested `Parameter`, or `alternative_value` if unknown
        """
        if alternative_value is None:
            alternative_value = Parameter(name, Parameter.Type.NOT_SET)
        return self._parameters.get(name, alternative_value)

    def get_parameters_by_prefix(self, prefix):
        """
        Get parameters that have a given prefix in their names as a dictionary.

        The names which are used as keys in the returned dictionary have the prefix removed.
        For example, if you use the prefix "foo" and the parameters "foo.ping", "foo.pong"
        and "bar.baz" exist, then the returned dictionary will have the keys "ping" and "pong".
        Note that the parameter separator is also removed from the parameter name to create the
        keys.

        An empty string for the prefix will match all parameters.

        If no parameters with the prefix are found, an empty dictionary will be returned.

        @param   prefix  the prefix of the parameters to get
        @return          list of `Parameter` under the given prefix, with prefix removed from names
        """
        parameters_with_prefix = {}
        if prefix:
            prefix = prefix + PARAMETER_SEPARATOR_STRING
        prefix_len = len(prefix)
        for parameter_name in self._parameters:
            if parameter_name.startswith(prefix):
                parameters_with_prefix.update(
                    {parameter_name[prefix_len:]: self._parameters.get(parameter_name)})
        return parameters_with_prefix

    def set_parameters(self, parameter_list):
        """
        Set parameters for the node, and return the result for the set action.

        If any parameter in the list was not declared beforehand and undeclared parameters are not
        allowed for the node, this method will raise a ParameterNotDeclaredException exception.

        Parameters are set in the order they are declared in the list.
        If setting a parameter fails due to not being declared, then the
        parameters which have already been set will stay set, and no attempt will
        be made to set the parameters which come after.

        If undeclared parameters are allowed, then all the parameters will be implicitly
        declared before being set even if they were not declared beforehand.
        Parameter overrides are ignored by this method.

        If a callback was registered previously with `add_on_set_parameters_callback`, it
        will be called prior to setting the parameters for the node, once for each parameter.
        If the callback prevents a parameter from being set, then it will be reflected in the
        returned result; no exceptions will be raised in this case.

        If the value type of the parameter is NOT_SET, and the existing parameter type is
        something else, then the parameter will be implicitly undeclared.

        @param   parameter_list  list of `Parameter` to set
        @return                  list of `SetParametersResult`, one for each input

        @throws  ParameterNotDeclaredException  if undeclared parameters are not allowed,
                                                and any given parameter is unknown
        """
        return self._set_parameters(parameter_list)

    def _set_parameters(self, parameter_list, descriptors=None, raise_on_failure=False,
                        allow_undeclared_parameters=False):
        """
        Set parameters for the node, and return the result for the set action.

        Method for internal usage; applies a setter method for each parameters in the list.
        By default it checks if the parameters were declared, raising an exception if at least
        one of them was not.

        If a callback was registered previously with `add_on_set_parameters_callback`, it
        will be called prior to setting the parameters for the node, once for each parameter.
        If the callback doesn't succeed for a given parameter, it won't be set and either an
        unsuccessful result will be returned for that parameter, or an exception will be raised
        according to `raise_on_failure` flag.

        @param   parameter_list               list of `Parameter` to set.
        @param   descriptors                  descriptors to set to the given parameters,
                                              as `{name: ParameterDescriptor}`,
                                              either one for each parameter or none at all
        @param   raise_on_failure             raise `InvalidParameterValueException`
                                              if any set-parameters callback rejects a parameter
        @param   allow_undeclared_parameters  auto-declare unknown parameters
        @return                               list of `SetParametersResult`, one for each input

        @throws  InvalidParameterValueException  if any set-parameters callback rejects a parameter
                                                 and `raise_on_failure` is true
        @throws  ParameterNotDeclaredException   if undeclared parameters are not allowed,
                                                 and any given parameter is unknown
        """
        if descriptors is not None:
            assert all(parameter.name in descriptors for parameter in parameter_list)

        results = []
        for param in parameter_list:
            if not allow_undeclared_parameters:
                self._check_undeclared_parameters([param])
            # If undeclared parameters are allowed, parameters with type `NOT_SET` shall be stored.
            result = self._set_parameters_atomically(
                [param],
                descriptors,
                allow_not_set_type=allow_undeclared_parameters
            )
            if raise_on_failure and not result.successful:
                raise exceptions.InvalidParameterValueException(param.name, param.value, result.reason)
            results.append(result)
        return results

    def set_parameters_atomically(self, parameter_list):
        """
        Set the given parameters, all at one time, and then aggregate result.

        If any parameter in the list was not declared beforehand and undeclared parameters are not
        allowed for the node, this method will raise a `ParameterNotDeclaredException` exception.

        Parameters are set all at once.
        If setting a parameter fails due to not being declared, then no parameter will be set.
        Either all of the parameters are set or none of them are set.

        If undeclared parameters are allowed for the node, then all the parameters will be
        implicitly declared before being set even if they were not declared beforehand.

        If a callback was registered previously with `add_on_set_parameters_callback`, it
        will be called prior to setting the parameters for the node only once for all parameters.
        If the callback prevents the parameters from being set, then it will be reflected in the
        returned result; no exceptions will be raised in this case.

        If the value type of the parameter is `NOT_SET`, and the existing parameter type is
        something else, then the parameter will be implicitly undeclared.

        @param   parameter_list  list of `Parameter` to set
        @return                  a single `SetParametersResult` for all parameters

        @throws  ParameterNotDeclaredException   if undeclared parameters are not allowed,
                                                 and any named parameter is unknown
        """
        self._check_undeclared_parameters(parameter_list)
        return self._set_parameters_atomically(parameter_list)

    def _check_undeclared_parameters(self, parameter_list):
        """
        Check if all parameters in list have correct types and were declared beforehand.

        @param   parameter_list                  list of `Parameter` to check
        @throws  ParameterNotDeclaredException   if any given parameter is unknown
        """
        if not all(isinstance(parameter, Parameter) for parameter in parameter_list):
            raise TypeError("parameter must be instance of type '{}'".format(repr(Parameter)))

        undeclared_parameters = (
            param.name for param in parameter_list if param.name not in self._parameters
        )
        if (not self._allow_undeclared_parameters and any(undeclared_parameters)):
            raise exceptions.ParameterNotDeclaredException(list(undeclared_parameters))

    def _set_parameters_atomically(self, parameter_list, descriptors=None,
                                   allow_not_set_type=False):
        """
        Set the given parameters, all at one time, and then aggregate result.

        This internal method does not reject undeclared parameters.
        If `allow_not_set_type` is False, a parameter with type `NOT_SET` will be undeclared.

        If a callback was registered previously with `add_on_set_parameters_callback`, it
        will be called prior to setting the parameters for the node only once for all parameters.
        If the callback prevents the parameters from being set, then it will be reflected in the
        returned result; no exceptions will be raised in this case.

        @param   parameter_list      list of `Parameter` to set
        @param   descriptors         new descriptors to apply to the parameters before setting them,
                                     as `{name: ParameterDescriptor}`,
                                     one for each parameter or none at all
        @param   allow_not_set_type  whether to store parameters of `NOT_SET` type,
                                     or undeclare them if False
        @return                      a single `SetParametersResult` for all parameters
        """
        if descriptors is not None:
            # If new descriptors are provided, ensure every parameter has an assigned descriptor
            # and do not check for read-only.
            assert all(parameter.name in descriptors for parameter in parameter_list)
            result = self._apply_descriptors(parameter_list, descriptors, False)
        else:
            # If new descriptors are not provided, use existing ones and check for read-only.
            result = self._apply_descriptors(parameter_list, self._descriptors, True)

        if not result.successful:
            return result
        elif self._parameters_callbacks:
            for callback in self._parameters_callbacks:
                result = callback(parameter_list)
                if not result.successful:
                    return result
        result = SetParametersResult(successful=True)

        if result.successful:
            for param in parameter_list:
                # If parameters without type and value are not allowed, they shall be undeclared.
                if not allow_not_set_type and Parameter.Type.NOT_SET == param.type_:
                    # Parameter deleted. (Parameter had value and new value is not set).
                    # Delete any unset parameters regardless of their previous value.
                    if param.name in self._parameters:
                        del self._parameters[param.name]
                    if param.name in self._descriptors:
                        del self._descriptors[param.name]
                    ros1.delete_param(param.name)
                else:
                    # Update descriptors; set a default if it doesn't exist.
                    # Don't update if it already exists for the current parameter and a new one
                    # was not specified in this method call.
                    if descriptors is not None:
                        self._descriptors[param.name] = descriptors[param.name]
                    elif param.name not in self._descriptors:
                        self._descriptors[param.name] = ParameterDescriptor()

                    # Descriptors have already been applied by this point.
                    self._parameters[param.name] = param
                    ros1.set_param(param.name, param.value)

        return result

    def add_on_set_parameters_callback(self, callback):
        """
        Add a callback in front to the list of callbacks.

        @param   callback  function to call with (`[Parameter]`) when setting any node parameters,
                           returning `SetParametersResult`
        """
        self._parameters_callbacks.insert(0, callback)

    def remove_on_set_parameters_callback(self, callback):
        """
        Remove a callback from list of callbacks.

        @param   callback    function registered with `add_on_set_parameters_callback()`
                             or `set_parameters_callback()`
        @throws  ValueError  if the callback is unknown
        """
        self._parameters_callbacks.remove(callback)

    def _apply_descriptors(self, parameter_list, descriptors, check_read_only=True):
        """
        Apply descriptors to parameters and return an aggregated result without saving parameters.

        In case no descriptors are provided to the method, existing descriptors shall be used.
        In any case, if a given parameter doesn't have a descriptor it shall be skipped.

        @param   parameter_list   list of `Parameter` to be checked
        @param   descriptors      `{name: `ParameterDescriptor}` to apply
        @param   check_read_only  whether to check for read-only descriptors
        @return                   a single `SetParametersResult` for all checks

        @throws  ParameterNotDeclaredException  if a descriptor is not provided,
                                                a given parameter is unknown
                                                and undeclared parameters are not allowed
        """
        for param in parameter_list:
            if param.name in descriptors:
                result = self._apply_descriptor(param, descriptors[param.name], check_read_only)
                if not result.successful:
                    return result
        return SetParametersResult(successful=True)

    def _apply_descriptor(self, parameter, descriptor=None, check_read_only=True):
        """
        Apply a descriptor to a parameter and return a result without saving the parameter.

        This method sets the type in the descriptor to match the parameter type.
        If a descriptor is provided, its name will be set to the name of the parameter.

        @param   parameter        `Parameter` to be checked
        @param   descriptor       `ParameterDescriptor` to apply,
                                  or None to use the stored descriptor
        @param   check_read_only  whether to check for read-only descriptor
        @return                   a `SetParametersResult` for all checks

        @throws  ParameterNotDeclaredException  if descriptor is not provided,
                                                the given parameter is unknown
                                                and undeclared parameters are not allowed
        """
        if descriptor is None:
            descriptor = self.describe_parameter(parameter.name)
        else:
            descriptor.name = parameter.name

        # The type in the descriptor has to match the type of the parameter.
        descriptor.type = parameter.type_.value

        if check_read_only and descriptor.read_only:
            return SetParametersResult(
                successful=False,
                reason='Trying to set a read-only parameter: {}.'.format(parameter.name))

        if parameter.type_ == Parameter.Type.INTEGER and descriptor.integer_range:
            return self._apply_integer_range(parameter, descriptor.integer_range[0])

        if parameter.type_ == Parameter.Type.DOUBLE and descriptor.floating_point_range:
            return self._apply_floating_point_range(parameter, descriptor.floating_point_range[0])

        return SetParametersResult(successful=True)

    def _apply_integer_range(self, parameter, integer_range):
        """
        Returns whether parameter value passes integer range check.

        Value must fall into given min-max range, endpoints included,
        and be an exact number of steps from range start if range has step defined.

        @param   parameter      `Parameter` to check
        @param   integer_range  `IntegerRange` to use
        @return                 `SetParametersResult` with success or failure reason
        """
        min_value = min(integer_range.from_value, integer_range.to_value)
        max_value = max(integer_range.from_value, integer_range.to_value)

        # Values in the edge are always OK.
        if parameter.value == min_value or parameter.value == max_value:
            return SetParametersResult(successful=True)

        if not min_value < parameter.value < max_value:
            return SetParametersResult(
                successful=False,
                reason='Parameter {} out of range. '
                       'Min: {}, Max: {}, value: {}'.format(
                            parameter.name, min_value, max_value, parameter.value
                        )
            )

        if integer_range.step != 0 and (parameter.value - min_value) % integer_range.step != 0:
            return SetParametersResult(
                successful=False,
                reason='The parameter value for {} is not a valid step. '
                       'Min: {}, max: {}, value: {}, step: {}'.format(
                            parameter.name,
                            min_value,
                            max_value,
                            parameter.value,
                            integer_range.step
                        )
            )

        return SetParametersResult(successful=True)

    def _apply_floating_point_range(self, parameter, floating_point_range):
        """
        Returns whether parameter value passes floating-point range check.

        Value must fall into given min-max range, endpoints included,
        and be a tolerably exact number of steps from range start if range has step defined.

        @param   parameter             `Parameter` to check
        @param   floating_point_range  `FloatingPointRange` to use
        @return                        `SetParametersResult` with success or failure reason
        """
        min_value = min(floating_point_range.from_value, floating_point_range.to_value)
        max_value = max(floating_point_range.from_value, floating_point_range.to_value)

        # Values in the edge are always OK.
        if (
            math.isclose(parameter.value, min_value, rel_tol=self.PARAM_REL_TOL) or
            math.isclose(parameter.value, max_value, rel_tol=self.PARAM_REL_TOL)
        ):
            return SetParametersResult(successful=True)

        if not min_value < parameter.value < max_value:
            return SetParametersResult(
                successful=False,
                reason='Parameter {} out of range '
                       'Min: {}, Max: {}, value: {}'.format(
                            parameter.name, min_value, max_value, parameter.value
                        )
            )

        if floating_point_range.step != 0.0:
            distance_int_steps = round((parameter.value - min_value) / floating_point_range.step)
            if not math.isclose(
                min_value + distance_int_steps * floating_point_range.step,
                parameter.value,
                rel_tol=self.PARAM_REL_TOL
            ):
                return SetParametersResult(
                    successful=False,
                    reason='The parameter value for {} is not close enough to a valid step. '
                           'Min: {}, max: {}, value: {}, step: {}'.format(
                                parameter.name,
                                min_value,
                                max_value,
                                parameter.value,
                                floating_point_range.step
                            )
                )

        return SetParametersResult(successful=True)

    def _apply_descriptor_and_set(self, parameter, descriptor=None, check_read_only=True):
        """
        Apply parameter descriptor and set parameter if successful.

        @param   parameter        `Parameter` to be checked
        @param   descriptor       `ParameterDescriptor` to apply,
                                  or None to use the stored descriptor
        @param   check_read_only  whether to check for read-only descriptors
        """
        result = self._apply_descriptor(parameter, descriptor, check_read_only)
        if result.successful:
            self._parameters[parameter.name] = parameter
        return result

    def describe_parameter(self, name):
        """
        Get the parameter descriptor of a given parameter.

        @param   name  fully-qualified name of the parameter, including its namespace
        @return        `ParameterDescriptor` corresponding to the parameter,
                        or default `ParameterDescriptor` if parameter is unknown
                        and undeclared parameters are allowed

        @throws  ParameterNotDeclaredException  if parameter is unknown
                                                and undeclared parameters are not allowed
        """
        try:
            return self._descriptors[name]
        except KeyError:
            if self._allow_undeclared_parameters:
                return ParameterDescriptor()
            else:
                raise exceptions.ParameterNotDeclaredException(name)

    def describe_parameters(self, names):
        """
        Get the parameter descriptors of a given list of parameters.

        @param   name  list of fully-qualified names of the parameters to describe
        @return        list of `ParameterDescriptor` corresponding to the given parameters.
                       Default `ParameterDescriptor` shall be returned for parameters that
                       had not been declared before if undeclared parameters are allowed.

        @throws  ParameterNotDeclaredException  if any parameter is unknown
                                                and undeclared parameters are not allowed
        """
        parameter_descriptors = []
        for name in names:
            parameter_descriptors.append(self.describe_parameter(name))
        return parameter_descriptors

    def set_descriptor(self, name, descriptor, alternative_value=None):
        """
        Set a new descriptor for a given parameter.

        The name in the descriptor is ignored and set to given `name`.

        @param   name               fully-qualified name of the parameter to set the descriptor to
        @param   descriptor         new `ParameterDescriptor` to apply to the parameter
        @param   alternative_value  `ParameterValue` to set to the parameter
                                    if the existing value does not comply with the new descriptor
        @return                     `ParameterValue` for the given parameter

        @throws  ParameterNotDeclaredException  if the parameter is unknown
                                                and undeclared parameters are not allowed
        @throws  ParameterImmutableException    if the parameter exists and is read-only
        @throws  ParameterValueException        if neither the existing value nor the alternative value
                                                comply with the provided descriptor
        """
        if not self.has_parameter(name):
            if not self._allow_undeclared_parameters:
                raise exceptions.ParameterNotDeclaredException(name)
            else:
                return self.get_parameter(name).get_parameter_value()

        if self.describe_parameter(name).read_only:
            raise exceptions.ParameterImmutableException(name)

        current_parameter = self.get_parameter(name)
        if alternative_value is None:
            alternative_parameter = current_parameter
        else:
            alternative_parameter = Parameter(name=name, value=alternative_value.get_value())

        # First try keeping the parameter, then try the alternative one.
        # Don't check for read-only since we are applying a new descriptor now.
        if not self._apply_descriptor_and_set(current_parameter, descriptor, False).successful:
            alternative_set_result = (
                self._apply_descriptor_and_set(alternative_parameter, descriptor, False)
            )
            if not alternative_set_result.successful:
                raise exceptions.InvalidParameterValueException(
                    name,
                    alternative_parameter.value,
                    alternative_set_result.reason
                )

        self._descriptors[name] = descriptor
        return self.get_parameter(name).get_parameter_value()

    def set_parameters_callback(self, callback):
        """
        Register a set parameters callback.

        @deprecated  Since Foxy. Use `add_on_set_parameters_callback()` instead.

        Calling this function will add a callback to the self._parameter_callbacks list.

        @param   callback  function to call with `([`Parameter`])` when setting any node parameters,
                           returning `SetParametersResult`
        """
        warnings.warn(
            'set_parameters_callback() is deprecated. '
            'Use add_on_set_parameters_callback() instead'
        )
        self._parameters_callbacks = [callback]

    def add_waitable(self, waitable):
        """Does nothing (ROS2 API compatibility stand-in)."""

    def remove_waitable(self, waitable):
        """Does nothing (ROS2 API compatibility stand-in)."""

    def create_publisher(self, msg_type, topic, qos_profile,
                         *, callback_group=None, event_callbacks=None):
        """
        Creates a new publisher.

        @param   msg_type         type class of ROS messages the publisher will publish
        @param   topic            the name of the topic the publisher will publish to
        @param   qos_profile      a `QoSProfile` or a queue size to apply to the publisher,
                                  publisher will be latched if `durability` is `TRANSIENT_LOCAL`
        @param   callback_group   ignored (ROS2 API compatibility stand-in)
        @param   event_callbacks  ignored (ROS2 API compatibility stand-in)
        @return                   `rospy.Publisher` instance
        """
        queue_size = qos_profile if isinstance(qos_profile, int) else qos_profile.depth
        latch = False
        if not isinstance(qos_profile, int):
            latch = (DurabilityPolicy.TRANSIENT_LOCAL == qos_profile.durability)
        pub = ros1.create_publisher(topic, msg_type, latch=latch, queue_size=queue_size)
        self.__publishers.append(pub)
        return pub

    def create_subscription(self, msg_type, topic, callback, qos_profile,
                            *, callback_group=None, event_callbacks=None, raw=False):
        """
        Creates a new subscription.

        @param   msg_type         type class of ROS messages the subscription will subscribe to.
        @param   topic            the name of the topic the subscription will subscribe to
        @param   callback         a user-defined callback function that is called when a message is
                                  received by the subscription
        @param   qos_profile      a `QoSProfile` or a queue size to apply to the subscription
        @param   callback_group   ignored (ROS2 API compatibility stand-in)
        @param   event_callbacks  ignored (ROS2 API compatibility stand-in)
        @param   raw              if `True`, then subscription will be done with `rospy.AnyMsg`,
                                  providing message raw binary bytes
        @return                   `rospy.Subscriber` instance
        """
        queue_size = qos_profile if isinstance(qos_profile, int) else qos_profile.depth
        sub = ros1.create_subscriber(topic, msg_type, callback=callback,
                                     queue_size=queue_size, raw=raw)
        self.__subscriptions.append(sub)
        return sub

    def create_client(self, srv_type, srv_name, *, qos_profile=None, callback_group=None):
        """
        Creates a new service client.

        @param   srv_type        type class of the service, like `std_srvs.srv.Trigger`
        @param   srv_name        the name of the service
        @param   qos_profile     ignored (ROS2 API compatibility stand-in)
        @param   callback_group  ignored (ROS2 API compatibility stand-in)
        @return                  `rospy.ServiceProxy` instance
        """
        client = ros1.create_client(srv_name, srv_type)
        self.__clients.append(client)
        return client

    def create_service(self, srv_type, srv_name, callback,
                       *, qos_profile=None, callback_group=None):
        """
        Creates a new service server.

        @param   srv_type        type class of the service, like `std_srvs.srv.Trigger`
        @param   srv_name        the name of the service
        @param   callback        a user-defined callback function that is called when a service request
                                 is received by the server, as callback(request, response)
        @param   qos_profile     ignored (ROS2 API compatibility stand-in)
        @param   callback_group  ignored (ROS2 API compatibility stand-in)
        @return                  `rospy.Service` instance
        """
        service = ros1.create_service(srv_name, srv_type, callback)
        self.__services.append(service)
        return service

    def create_timer(self, timer_period_sec, callback, callback_group=None, clock=None):
        """
        Creates a new timer.

        The timer will be started and every `timer_period_sec` number of seconds the provided
        callback function will be called.

        @param   timer_period_sec  the period (s) of the timer
        @param   callback          a user-defined callback function that is called when the timer expires
        @param   callback_group    ignored (ROS2 API compatibility stand-in)
        @param   clock             the clock which the timer gets time from
        @return                    `rospy.Timer` instance
        """
        timer = ros1.create_timer(timer_period_sec, callback)
        timer._clock = clock or self._clock
        self.__timers.append(timer)
        return timer

    def create_guard_condition(self, callback, callback_group=None):
        """Does nothing and returns None (ROS2 API compatibility stand-in)."""
        return None

    def create_rate(self, frequency, clock=None):
        """
        Creates a rate object.

        The timer will be started and every `timer_period_sec` number of seconds the provided
        callback function will be called.

        @param   frequency  the frequency the Rate runs at (Hz)
        @param   clock      ignored (ROS2 API compatibility stand-in)
        @return             `rospy.Rate` instance
        """
        if frequency <= 0:
            raise ValueError('frequency must be > 0')
        return ros1.create_rate(frequency)

    def destroy_publisher(self, publisher):
        """
        Destroy a publisher created by the node.

        @return  `True` if successful, `False` otherwise
        """
        publisher.unregister()
        if publisher in self.__publishers:
            self.__publishers.remove(publisher)
            return True
        return False

    def destroy_subscription(self, subscription):
        """
        Destroy a subscription created by the node.

        @return  `True` if successful, `False` otherwise
        """
        subscription.unregister()
        if subscription in self.__subscriptions:
            self.__subscriptions.remove(subscription)
            return True
        return False

    def destroy_client(self, client):
        """
        Destroy a service client created by the node.

        @return  `True` if successful, `False` otherwise
        """
        client.close()
        if client in self.__clients:
            self.__clients.remove(client)
            return True
        return False

    def destroy_service(self, service):
        """
        Destroy a service server created by the node.

        @return  `True` if successful, `False` otherwise
        """
        service.shutdown()
        if service in self.__services:
            self.__services.remove(service)
            return True
        return False

    def destroy_timer(self, timer):
        """
        Destroy a timer created by the node.

        @return  `True` if successful, `False` otherwise
        """
        timer.shutdown()
        if timer in self.__timers:
            self.__timers.remove(timer)
        return False

    def destroy_guard_condition(self, guard):
        """
        Does nothing (ROS2 API compatibility stand-in).

        :return: ``True``
        """
        return True

    def destroy_rate(self, rate):
        """
        Does nothing (ROS2 API compatibility stand-in).

        :return: `True`
        """
        return True

    def destroy_node(self):
        """
        Destroys the node and shuts down rospy.

        Frees resources used by the node, including any entities created by the following methods:

        * `create_publisher`
        * `create_subscription`
        * `create_client`
        * `create_service`
        * `create_timer`

        """
        while self.__publishers:
            self.__publishers.pop().unregister()
        while self.__subscriptions:
            self.__subscriptions.pop().unregister()
        while self.__clients:
            self.__clients.pop().close()
        while self.__services:
            self.__services.pop().shutdown()
        while self.__timers:
            self.__timers.pop().shutdown()
        rospy.signal_shutdown(reason="")

    def get_publisher_names_and_types_by_node(self, node_name, node_namespace, no_demangle=False):
        """
        Gets a list of discovered topics for publishers of a remote node.

        @param   node_name       name of a remote node to get publishers for
        @param   node_namespace  namespace of the remote node
        @param   no_demangle     ignored (ROS2 API compatibility stand-in)
        @return                  list of tuples, the first element of each tuple is the topic name
                                 and the second element is a list of topic types

        @throws  NodeNameNonExistentError  if the node wasn't found
        @throws  RuntimeError              unexpected failure
        """
        node_topics = {}
        node_name = util.namejoin(node_namespace, node_name)
        # getSystemState() returns [[topic, [publishing node, ]], [..subs], [..services]]
        try: state = ros1.MASTER.getSystemState()[-1]
        except Exception as e: raise RuntimeError(e)
        for topic, nodes in state[0]:
            if node_name in nodes:
                node_topics.setdefault(topic, [])
        if not node_topics and not any(node_name in nn for lst in state[1:] for _, nn in lst):
            raise NodeNameNonExistentError(
                "Cannot get publisher names and types for nonexistent node", node_name)
        try: topics_types = ros1.MASTER.getTopicTypes()[-1] if node_topics else ()
        except Exception as e: raise RuntimeError(e)
        for topic, typename in topics_types:
            if topic in node_topics:
                node_topics[topic].append(typename)
        return sorted((t, sorted(nn)) for t, nn in node_topics.items())


    def get_subscriber_names_and_types_by_node(self, node_name, node_namespace, no_demangle=False):
        """
        Gets a list of discovered topics for subscriptions of a remote node.

        @param   node_name       name of a remote node to get subscriptions for
        @param   node_namespace  namespace of the remote node
        @param   no_demangle     ignored (ROS2 API compatibility stand-in)
        @return                  list of tuples, the first element of each tuple is the topic name
                                 and the second element is a list of topic types

        @throws  NodeNameNonExistentError  if the node wasn't found
        @throws  RuntimeError              unexpected failure
        """
        node_topics = {}
        node_name = util.namejoin(node_namespace, node_name)
        # getSystemState() returns [[topic, [publishing node, ]], [..subs], [..services]]
        try: state = ros1.MASTER.getSystemState()[-1]
        except Exception as e: raise RuntimeError(e)
        for topic, nodes in state[1]:
            if node_name in nodes:
                node_topics.setdefault(topic, [])
        if not node_topics and not any(node_name in nn for lst in state[::2] for _, nn in lst):
            raise NodeNameNonExistentError(
                "Cannot get subscriber names and types for nonexistent node", node_name)
        try: topics_types = ros1.MASTER.getTopicTypes()[-1] if node_topics else ()
        except Exception as e: raise RuntimeError(e)
        for topic, typename in topics_types:
            if topic in node_topics:
                node_topics[topic].append(typename)
        return sorted((t, sorted(nn)) for t, nn in node_topics.items())


    def get_service_names_and_types_by_node(self, node_name, node_namespace):
        """
        Gets a list of discovered service server topics for a remote node.

        @param   node_name       name of a remote node to get services for
        @param   node_namespace  namespace of the remote node
        @return                  list of tuples,
                                 the first element of each tuple is the service server name
                                 and the second element is a list of service types

        @throws  NodeNameNonExistentError  if the node wasn't found
        @throws  RuntimeError              unexpected failure
        """
        node_services = {}
        node_name = util.namejoin(node_namespace, node_name)
        # getSystemState() returns [[topic, [publishing node, ]], [..subs], [..services]]
        try: state = ros1.MASTER.getSystemState()[-1]
        except Exception as e: raise RuntimeError(e)
        for topic, nodes in state[2]:
            if node_name in nodes:
                node_services.setdefault(topic, [])
        if not node_services and not any(node_name in nn for lst in state[::2] for _, nn in lst):
            raise NodeNameNonExistentError(
                "Cannot get service names and types for nonexistent node", node_name)
        for service in list(node_services):
            try: node_services[service].append(api.make_full_typename(service, "srv"))
            except Exception as e: raise RuntimeError(e)
        return sorted((t, sorted(nn)) for t, nn in node_services.items())

    def get_client_names_and_types_by_node(self, node_name, node_namespace):
        """
        Returns an empty list (ROS2 API compatibility stand-in)

        ROS1 does not provide service clients information.

        @param   node_name       ignored
        @param   node_namespace  ignored
        @return                  `[]`
        """
        return []

    def get_topic_names_and_types(self, no_demangle=False):
        """
        Gets a list of available topic names and types.

        @param   no_demangle  ignored (ROS2 API compatibility stand-in)
        @return               list of tuples, the first element of each tuple is the topic name
                              and the second element is a list of topic types
        """
        return ros1.get_topics()

    def get_service_names_and_types(self):
        """
        Gets a list of available services.

        @return                  list of tuples, the first element of each tuple is the service name
                                 and the second element is a list of service types
        """
        return ros1.get_services()

    def get_node_names(self):
        """
        Get a list of names for discovered nodes.

        @return  list of node names without namespaces
        """
        return [util.namesplit(n)[-1] for n in ros1.get_nodes()]

    def get_node_names_and_namespaces(self):
        """
        Get a list of names and namespaces for discovered nodes.

        @return  list of tuples containing two strings: the node name and node namespace
        """
        return [util.namesplit(n)[::-1] for n in ros1.get_nodes()]

    def get_node_names_and_namespaces_with_enclaves(self):
        """
        Get a list of names, namespaces and enclaves for discovered nodes.

        @return  list of tuples containing three strings: the node name, node namespace
                 and enclave (the latter always "/" in ROS1)
        """
        return [util.namesplit(n)[::-1] + ("/", ) for n in ros1.get_nodes()]

    def count_publishers(self, topic_name):
        """
        Return the number of publishers on a given topic, globally.

        `topic_name` may be a relative, private, or fully qualifed topic name.
        A relative or private topic is expanded using this node's namespace and name.
        The queried topic name is not remapped.

        @param   topic_name  the topic_name on which to count the number of publishers
        @return              the number of publishers on the topic
        """
        topic_name = ros1.resolve_name(topic_name)
        # getSystemState() returns [[topic, [publishing node, ]], [..subs], [..services]]
        nodes = sum((nn for t, nn in ros1.MASTER.getSystemState()[-1][0] if t == topic_name), [])
        return len(nodes)

    def count_subscribers(self, topic_name):
        """
        Return the number of subscribers on a given topic.

        `topic_name` may be a relative, private, or fully qualifed topic name.
        A relative or private topic is expanded using this node's namespace and name.
        The queried topic name is not remapped.

        @param   topic_name  the topic_name on which to count the number of subscribers
        @return              the number of subscribers on the topic
        """
        topic_name = ros1.resolve_name(topic_name)
        # getSystemState() returns [[topic, [publishing node, ]], [..subs], [..services]]
        nodes = sum((nn for t, nn in ros1.MASTER.getSystemState()[-1][1] if t == topic_name), [])
        return len(nodes)

    def _get_info_by_topic(self, topic_name, endpoint_type, no_mangle):
        """
        Returns a list of `TopicEndpointInfo`.

        @param   endpoint_type  one of TopicEndpointTypeEnum
        @param   no_mangle      whether topic name should be used as is,
                                by default it will be namespaced under node and remapped
        """
        endpoints = []
        if not no_mangle:
            topic_name = ros1.remap_name(ros1.resolve_name(topic_name))
        IDX = 0 if TopicEndpointTypeEnum.PUBLISHER == endpoint_type else 1

        # getSystemState() returns [[topic, [publishing node, ]], [..subs], [..services]]
        nodes = sum((nn for t, nn in ros1.MASTER.getSystemState()[-1][IDX] if t == topic_name), [])
        if nodes:
            typename = next(nn[0] for t, nn in ros1.get_topics() if t == topic_name)
        for node_name in nodes:
            kws = dict(zip(("node_name", "node_namespace"), util.namesplit(node_name)))
            kws.update(topic_type=typename, endpoint_type=endpoint_type,
                       qos_profile=QoSPresetProfiles.SYSTEM_DEFAULT.value)
            endpoints.append(TopicEndpointInfo(**kws))
        return endpoints

    def get_publishers_info_by_topic(self, topic_name, no_mangle=False):
        """
        Return a list of publishers on a given topic.

        The returned parameter is a list of TopicEndpointInfo objects, where each will contain
        the node name, node namespace, topic type, topic endpoint's GID, and its QoS profile.

        `topic_name` may be a relative, private, or fully qualified topic name.
        A relative or private topic will be expanded using this node's namespace and name.
        The queried `topic_name` is not remapped.

        @param   topic_name  the topic_name on which to find the publishers
        @param   no_mangle   whether topic name should be used as is,
                             by default it will be namespaced under node and remapped
        @return              a list of `rosros.rclify.topic_endpoint_info.TopicEndpointInfo`
                             for all the publishers on this topic
        """
        return self._get_info_by_topic(topic_name, TopicEndpointTypeEnum.PUBLISHER, no_mangle)

    def get_subscriptions_info_by_topic(self, topic_name, no_mangle=False):
        """
        Return a list of subscriptions on a given topic.

        The returned parameter is a list of TopicEndpointInfo objects, where each will contain
        the node name, node namespace, topic type, topic endpoint's GID, and its QoS profile.

        When the `no_mangle` parameter is `true`, the provided `topic_name` should be a valid topic
        name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
        When the `no_mangle` parameter is `false`, the provided `topic_name` should follow
        ROS topic name conventions.

        `topic_name` may be a relative, private, or fully qualified topic name.
        A relative or private topic will be expanded using this node's namespace and name.
        The queried `topic_name` is not remapped.

        @param   topic_name  the topic_name on which to find the subscriptions
        @param   no_mangle   whether topic name should be used as is,
                             by default it will be namespaced under node and remapped
        @return              a list of `rosros.rclify.topic_endpoint_info.TopicEndpointInfo`
                             for all the subscriptions on this topic
        """
        return self._get_info_by_topic(topic_name, TopicEndpointTypeEnum.SUBSCRIPTION, no_mangle)


__all__ = [
    "PARAMETER_SEPARATOR_STRING", "FloatingPointRange", "IntegerRange",
    "NodeNameNonExistentError", "Node", "Parameter", "ParameterDescriptor", "ParameterValue",
    "SetParametersResult"
]
