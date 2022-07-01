#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test: general API functions.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     13.04.2022
@modified    01.07.2022
------------------------------------------------------------------------------
"""
import datetime
import decimal
import logging
import os
import sys

import std_msgs.msg
import std_srvs.srv

from rosros import api

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from test import testbase
from test.testbase import ROS1

logger = logging.getLogger()


class TestAPI(testbase.TestBase):
    """Tests general API."""

    ## Test name used in flow logging
    NAME = "test_api"


    def test_messages(self):
        """Tests API functions for dealing with messages."""
        NAME = lambda f: "%s.%s()" % (f.__module__, f.__name__)
        ERR  = lambda f: "Unexpected result from %s." % f.__name__

        func = api.get_message_class
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            self.assertEqual(func("std_msgs/Bool"),        std_msgs.msg.Bool, ERR(func))
            self.assertEqual(func("std_msgs/msg/Bool"),    std_msgs.msg.Bool, ERR(func))
            self.assertEqual(func("std_srvs/SetBool"),     std_srvs.srv.SetBool, ERR(func))
            self.assertEqual(func("std_srvs/srv/SetBool"), std_srvs.srv.SetBool, ERR(func))
            self.assertEqual(func(std_msgs.msg.Bool),      std_msgs.msg.Bool, ERR(func))
            self.assertEqual(func(std_msgs.msg.Bool()),    std_msgs.msg.Bool, ERR(func))
            srvcls = std_srvs.srv.SetBool
            reqcls, respcls = (srvcls._request_class, srvcls._response_class) if ROS1 else \
                              (srvcls.Request, srvcls.Response)
            self.assertEqual(func(reqcls),    reqcls,  ERR(func))
            self.assertEqual(func(reqcls()),  reqcls,  ERR(func))
            self.assertEqual(func(respcls()), respcls, ERR(func))
            self.assertEqual(func(""),        None,    ERR(func))

        func = api.get_message_definition
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            self.assertTrue(func("std_msgs/Bool"),            ERR(func))
            self.assertTrue(func("std_msgs/msg/Bool"),        ERR(func))
            self.assertTrue(func("std_srvs/SetBoolRequest"),  ERR(func))
            self.assertTrue(func("std_srvs/SetBoolResponse"), ERR(func))
            self.assertTrue(func(std_msgs.msg.Bool),          ERR(func))
            self.assertTrue(func(std_msgs.msg.Bool()),        ERR(func))

        func = api.get_message_fields
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            self.assertEqual(func(std_msgs.msg.Bool),   {"data": "bool"}, ERR(func))
            self.assertEqual(func(std_msgs.msg.Bool()), {"data": "bool"}, ERR(func))
            dct = {"seq": "uint32", "stamp": "time", "frame_id": "string"} if ROS1 else \
                  {"stamp": "builtin_interfaces/Time", "frame_id": "string"}
            self.assertEqual(func(std_msgs.msg.Header), dct, ERR(func))
            self.assertEqual(func(None),                {},  ERR(func))

        func = api.get_message_type
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            self.assertEqual(func(std_msgs.msg.Bool),   "std_msgs/Bool", ERR(func))
            self.assertEqual(func(std_msgs.msg.Bool()), "std_msgs/Bool", ERR(func))
            self.assertEqual(func(type(api.make_time())), "%s/Time" % api.FAMILY, ERR(func))
            self.assertEqual(func(api.make_time()),       "%s/Time" % api.FAMILY, ERR(func))
            self.assertEqual(func(type(api.make_duration())), "%s/Duration" % api.FAMILY, ERR(func))
            self.assertEqual(func(api.make_duration()),       "%s/Duration" % api.FAMILY, ERR(func))


        func = api.get_message_type_hash
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            HASH = "8b94c1b53db61fb6aed406028ad6332a"
            self.assertEqual(func(std_msgs.msg.Bool),   HASH, ERR(func))
            self.assertEqual(func(std_msgs.msg.Bool()), HASH, ERR(func))
            self.assertEqual(func("std_msgs/Bool"),     HASH, ERR(func))
            self.assertEqual(func("std_msgs/msg/Bool"), HASH, ERR(func))
            self.assertTrue(func("std_msgs/Header"),          ERR(func))

        func = api.get_message_value
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            msg1 = std_msgs.msg.Header(frame_id=self.NAME)
            msg2 = std_msgs.msg.UInt8MultiArray(data=b"123")
            self.assertEqual(func(msg1, "frame_id"), msg1.frame_id,   ERR(func))
            self.assertEqual(func(msg1, "stamp"),    msg1.stamp,      ERR(func))
            self.assertEqual(func(msg2, "data"),     list(msg2.data), ERR(func))
            with self.assertRaises(Exception, msg=ERR(func)):
                func(msg1, "nosuchfield")

        func = api.get_service_definition
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            self.assertTrue(func("std_srvs/SetBool"),     ERR(func))
            self.assertTrue(func("std_srvs/srv/Trigger"), ERR(func))
            self.assertTrue(func(std_srvs.srv.SetBool),   ERR(func))
            self.assertTrue(func(std_srvs.srv.SetBool()), ERR(func)) if ROS1 else None

        func = api.get_service_request_class
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            expected = std_srvs.srv.SetBool._request_class if ROS1 else \
                       std_srvs.srv.SetBool.Request
            self.assertEqual(func("std_srvs/SetBool"),     expected, ERR(func))
            self.assertEqual(func("std_srvs/srv/SetBool"), expected, ERR(func))
            self.assertEqual(func(std_srvs.srv.SetBool),   expected, ERR(func))
            self.assertEqual(func(std_srvs.srv.SetBool()), expected, ERR(func)) if ROS1 else None

        func = api.get_service_response_class
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            expected = std_srvs.srv.SetBool._response_class if ROS1 else \
                       std_srvs.srv.SetBool.Response
            self.assertEqual(func("std_srvs/SetBool"),     expected, ERR(func))
            self.assertEqual(func("std_srvs/srv/SetBool"), expected, ERR(func))
            self.assertEqual(func(std_srvs.srv.SetBool),   expected, ERR(func))
            self.assertEqual(func(std_srvs.srv.SetBool()), expected, ERR(func)) if ROS1 else None

        func = api.is_ros_message
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            msg = std_msgs.msg.Header(frame_id=self.NAME)
            self.assertTrue(func(msg),       ERR(func))
            self.assertTrue(func(msg.stamp), ERR(func))
            self.assertFalse(func(func),     ERR(func))

        func = api.is_ros_service
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            srvcls = std_srvs.srv.SetBool
            self.assertTrue(func(srvcls), ERR(func))
            self.assertFalse(func(func),  ERR(func))

        func = api.dict_to_message
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            dct = {"seq": 3, "stamp": {"secs": 1, "nsecs": 2}, "frame_id": self.NAME} if ROS1 else \
                  {"stamp": {"sec": 1, "nanosec": 2}, "frame_id": self.NAME}
            stamp = api.make_time(1, 2)
            msg = std_msgs.msg.Header(frame_id=self.NAME, stamp=stamp if ROS1 else stamp.to_msg())
            if ROS1: msg.seq = dct["seq"]
            self.assertEqual(func(dct, std_msgs.msg.Header()), msg, ERR(func))

        func = api.message_to_dict
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            dct = {"seq": 3, "stamp": {"secs": 1, "nsecs": 2}, "frame_id": self.NAME} if ROS1 else \
                  {"stamp": {"sec": 1, "nanosec": 2}, "frame_id": self.NAME}
            stamp = api.make_time(1, 2)
            msg = std_msgs.msg.Header(frame_id=self.NAME, stamp=stamp if ROS1 else stamp.to_msg())
            if ROS1: msg.seq = dct["seq"]
            self.assertEqual(func(msg), dct, ERR(func))

        func = api.message_to_str
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            stamp = api.make_time(1, 2)
            msg  = std_msgs.msg.Header(frame_id=self.NAME, stamp=stamp if ROS1 else stamp.to_msg())
            if ROS1:
                exp1 = "std_msgs.msg.Header(seq=0, stamp=rospy.Time(secs=1, nsecs=2), " \
                                           "frame_id='%s')" % self.NAME
                exp2 = """std_msgs.msg.Header(
  seq=0,
  stamp=rospy.Time(
    secs=1,
    nsecs=2
  ),
  frame_id='%s'
)""" % self.NAME
            else:
                exp1 = "std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1, nanosec=2), " \
                                           "frame_id='%s')" % self.NAME
                exp2 = """std_msgs.msg.Header(
  stamp=builtin_interfaces.msg.Time(
    sec=1,
    nanosec=2
  ),
  frame_id='%s'
)""" % self.NAME

            self.assertEqual(func(msg),              exp1, ERR(func))
            self.assertEqual(func(msg, indent=2),    exp2, ERR(func))
            self.assertEqual(func(msg, indent="  "), exp2, ERR(func))

        func = api.serialize_message
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            expected = b"\x01" if ROS1 else b"\x00\x01\x00\x00\x01"
            self.assertEqual(func(std_msgs.msg.Bool(data=True)), expected, ERR(func))

        func = api.deserialize_message
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            binary = b"\x01" if ROS1 else b"\x00\x01\x00\x00\x01"
            expected = std_msgs.msg.Bool(data=True)
            self.assertEqual(func(binary, "std_msgs/Bool"),     expected, ERR(func))
            self.assertEqual(func(binary, "std_msgs/msg/Bool"), expected, ERR(func))
            self.assertEqual(func(binary, std_msgs.msg.Bool),   expected, ERR(func))
            self.assertEqual(func(binary, std_msgs.msg.Bool()), expected, ERR(func))


    def test_typenames(self):
        """Tests API functions for dealing with ROS types."""
        NAME = lambda f: "%s.%s()" % (f.__module__, f.__name__)
        ERR  = lambda f: "Unexpected result from %s." % f.__name__

        func = api.canonical
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            self.assertEqual(func("std_msgs/msg/Bool"),    "std_msgs/Bool",    ERR(func))
            self.assertEqual(func("std_srvs/srv/SetBool"), "std_srvs/SetBool", ERR(func))
            self.assertEqual(func("pkg/Cls"),              "pkg/Cls",          ERR(func))
            self.assertEqual(func("Cls"),                  "Cls",              ERR(func))

        func = api.format_param_name
        with self.subTest(NAME):
            logger.info("Testing %s.", NAME(func))
            SEP = api.PARAM_SEPARATOR
            self.assertEqual(func("/slashy/param"), SEP.join(("slashy", "param")), ERR(func))
            self.assertEqual(func("/dotty/param"),  SEP.join(("dotty", "param")),  ERR(func))
            self.assertEqual(func("%sprivate/param" % api.PRIVATE_PREFIX),
                             SEP.join(("private", "param")),                       ERR(func))

        func = api.get_type_alias
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            signed, unsigned = ("byte", "char") if ROS1 else ("char", "byte")
            self.assertEqual(func("int8"),  signed,   ERR(func))
            self.assertEqual(func("uint8"), unsigned, ERR(func))
            self.assertEqual(func("int16"), None,     ERR(func))

        func = api.get_alias_type
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            signed, unsigned = ("byte", "char") if ROS1 else ("char", "byte")
            self.assertEqual(func(signed),   "int8",  ERR(func))
            self.assertEqual(func(unsigned), "uint8", ERR(func))
            self.assertEqual(func("other"),  None,    ERR(func))

        func = api.make_full_typename
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            self.assertEqual(func("std_msgs/Bool"),               "std_msgs/msg/Bool", ERR(func))
            self.assertEqual(func("std_msgs/msg/Bool"),           "std_msgs/msg/Bool", ERR(func))
            self.assertEqual(func("std_srvs/SetBool", "srv"),     "std_srvs/srv/SetBool", ERR(func))
            self.assertEqual(func("std_srvs/srv/SetBool", "srv"), "std_srvs/srv/SetBool", ERR(func))
            self.assertEqual(func("%s/Time" % api.FAMILY),
                             "%s/Time" % api.FAMILY, ERR(func))
            self.assertEqual(func("%s/Duration" % api.FAMILY),
                             "%s/Duration" % api.FAMILY, ERR(func))

        func = api.scalar
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            self.assertEqual(func("int8"), "int8", ERR(func))
            self.assertEqual(func("std_msgs/Bool[]"), "std_msgs/Bool", ERR(func))


    def test_temporals(self):
        """Tests API functions for dealing with ROS time/duration values."""
        NAME = lambda f: "%s.%s()" % (f.__module__, f.__name__)
        ERR  = lambda f: "Unexpected result from %s." % f.__name__

        func = api.time_category
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            for typename in api.ROS_TIME_TYPES:
                expected = "duration" if "duration" in typename.lower() else "time"
                self.assertEqual(func(typename), expected, ERR(func))
            for cls in api.ROS_TIME_CLASSES:
                expected = "duration" if "duration" in cls.__name__.lower() else "time"
                self.assertEqual(func(cls),   expected, ERR(func))
                self.assertEqual(func(cls()), expected, ERR(func))
            self.assertEqual(func(self), self, ERR(func))

        func = api.is_ros_time
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            msg = std_msgs.msg.Header(frame_id=self.NAME)
            self.assertTrue(func(type(msg.stamp)), ERR(func))
            self.assertTrue(func(msg.stamp),       ERR(func))
            self.assertFalse(func(msg),            ERR(func))

        func = api.make_duration
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            dur = func(1, 2)
            self.assertIsInstance(dur, tuple(api.ROS_TIME_CLASSES),  ERR(func))
            self.assertTrue(api.is_ros_time(type(dur)), ERR(func))
            self.assertTrue(api.is_ros_time(dur),       ERR(func))

        func = api.make_time
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            self.assertIsInstance(func(1, 2), tuple(api.ROS_TIME_CLASSES),  ERR(func))

        func = api.to_datetime
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            tval, dval = api.make_time(1234), api.make_duration(1234)
            expected = datetime.datetime.fromtimestamp(1234)
            self.assertEqual(func(tval), expected, ERR(func))
            self.assertEqual(func(dval), expected, ERR(func))
            self.assertEqual(func(666),       666, ERR(func))

        func = api.to_decimal
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            tval = api.make_time    (123456789, 987654321)
            dval = api.make_duration(123456789, 987654321)
            expected = decimal.Decimal("123456789.987654321")
            self.assertEqual(func(tval), expected, ERR(func))
            self.assertEqual(func(dval), expected, ERR(func))
            self.assertEqual(func(666),       666, ERR(func))

        func = api.to_nsec
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            tval = api.make_time    (123456789, 987654321)
            dval = api.make_duration(123456789, 987654321)
            expected = 123456789987654321
            self.assertEqual(func(tval), expected, ERR(func))
            self.assertEqual(func(dval), expected, ERR(func))
            self.assertEqual(func(666),       666, ERR(func))

        func = api.to_sec
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            tval = api.make_time    (1, 123456789)
            dval = api.make_duration(1, 123456789)
            expected = 1.123456789
            self.assertEqual(func(tval), expected, ERR(func))
            self.assertEqual(func(dval), expected, ERR(func))
            self.assertEqual(func(666),       666, ERR(func))

        func = api.to_sec_nsec
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            tval = api.make_time    (123456789, 987654321)
            dval = api.make_duration(123456789, 987654321)
            expected = (123456789, 987654321)
            self.assertEqual(func(tval), expected, ERR(func))
            self.assertEqual(func(dval), expected, ERR(func))
            self.assertEqual(func(666),       666, ERR(func))



if "__main__" == __name__:
    TestAPI.run_rostest()
