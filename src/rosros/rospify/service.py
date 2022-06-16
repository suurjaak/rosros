"""
Stand-ins for `rospy.impl.tcpros_service` in ROS2.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     30.05.2022
@modified    30.05.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rospify.service
import abc
import sys
import time

import rclpy.client

from .. import ros2
from . import exceptions


def wait_for_service(service, timeout=None):
    """
    Blocks until service is available.

    Use this in initialization code if your program depends on a service already running.

    @param   service  name of service
    @param   timeout  timeout time in seconds or Duration, None for no timeout.
                      NOTE: timeout=0 is invalid as wait_for_service actually
                      contacts the service, so non-blocking behavior is not possible.
                      For timeout=0 use cases, just call the service without waiting.

    @throws  ROSException           if specified timeout is exceeded
    @throws  ROSInterruptException  if shutdown interrupts wait
    """
    monostart, timeout = time.monotonic(), ros2.to_sec(timeout)
    if timeout == 0.:
        raise ValueError("timeout must be non-zero")
    deadline = (monostart + timeout) if timeout else 2**31 - 1
    service, timer, rate, cls, cli = ros2.resolve_name(service), None, None, None, None

    try:
        timer = ros2.create_timer(0.1, callback=None)
        rate = rclpy.timer.Rate(timer, context=ros2.NODE.context)

        while not cls and time.monotonic() < deadline:
            typename = next((tt[0] for n, tt in ros2.get_services() if n == service), None)
            if typename: cls = ros2.get_message_class(typename)
            else: rate.sleep()

        mononow = time.monotonic()
        if mononow >= deadline:
            raise exceptions.ROSException("timeout exceeded while waiting for service %s" % service)

        cli = ros2.create_client(service, cls)
        cli.wait_for_service(None if not timeout else timeout + (mononow - monostart))
    except rclpy.exceptions.ROSInterruptException as e:
        raise exceptions.ROSInterruptException(*e.args) from e
    finally:
        timer and ros2.destroy_entity(timer)
        cli and ros2.destroy_entity(cli)
    timer and ros2.destroy_entity(timer)


class Service(abc.ABC):
    """Stand-in for `rospy.Service`, wrapping the creation of ROS2 service."""

    def __new__(cls, name, service_class, handler,
                buff_size=None, error_handler=None):
        """
        Returns rclpy.service.Service instance.

        @param   name           service name, absolute or relative
        @param   service_class  service definition class
        @param   handler        callback function for processing service request.
                                Function takes in a ServiceRequest and returns a ServiceResponse
                                of the appropriate type. Function may also return a list, tuple,
                                or dictionary with arguments to initialize a ServiceResponse
                                instance of the correct type.
                                If handler cannot process request, it may either return None,
                                to indicate failure, or it may raise a rospify.ServiceException
                                to send a specific error message to the client. Returning None
                                is always considered a failure.
        @param   buff_size      ignored (ROS1 compatibility stand-in)
        @param   error_handler  callback function for handling errors raised in the service code,
                                as fn(exception, exception_type, exception_value, traceback)
        """
        if callable(error_handler):
            def wrapper(req):
                try: return realhandler(req)
                except Exception as e:
                    error_handler(*(e, ) + sys.exc_info())
                    raise
            handler, realhandler = wrapper, handler
        try:
            return ros2.create_service(name, service_class, handler)
        except Exception as e:
            raise exceptions.ROSException() from e

    @classmethod
    def __subclasshook__(cls, C):
        """Returns true if C is a ROS2 service server class, else `NotImplemented`."""
        return True if issubclass(C, rclpy.service.Service) else NotImplemented



class ServiceProxy(abc.ABC):
    """Stand-in for `rospy.ServiceProxy`, wrapping the creation of ROS2 service client."""

    def __new__(cls, name, service_class, persistent=None, headers=None):
        """
        Returns rclpy.client.Client instance.

        @param   name           name of service to call
        @param   service_class  auto-generated service class
        @param   persistent     ignored (ROS1 compatibility stand-in)
        @param   headers        ignored (ROS1 compatibility stand-in)
        """
        return ros2.create_client(name, service_class)

    @classmethod
    def __subclasshook__(cls, C):
        """Returns true if C is a ROS2 service client class, else `NotImplemented`."""
        return True if issubclass(C, rclpy.client.Client) else NotImplemented


class ServiceException(Exception):
    """Exception class for service-related errors"""


__all__ = [
    "Service", "ServiceProxy", "wait_for_service"
]
