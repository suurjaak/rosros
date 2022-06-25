#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test: use rclify stand-in for rclpy in ROS1.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     01.06.2022
@modified    25.06.2022
------------------------------------------------------------------------------
"""
import functools
import inspect
import logging
import os
import sys
import threading
import time

import rosros
from rosros import rclify as rclpy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from test import testbase, testnode

logger = logging.getLogger()


class TestRclify(testbase.TestBase):
    """Tests using rclify stand-in for rclpy in ROS1."""

    ## Test and node name used in flow logging
    NAME = "test_rclify"

    ## Node namespace
    NAMESPACE = "/tests"


    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._pubs  = {}    # {topic name: publisher instance}
        self._subs  = {}    # {topic name: subscriber instance}
        self._clis  = {}    # {service name: service client instance}
        self._srvs  = {}    # {service name: service server instance}
        self._msgs  = {}    # {topic name: [incoming message, ]}
        self._reqs  = {}    # {service name: service request instance}
        self._exps  = {}    # {topic/service name: expected message data dictionary}
        self._types = {}    # {topic/service name: data class}
        self._node  = None  # rclpy.node.Node or rosros.rclify.node.Node
        self._exec  = None  # rclpy.executors.Executor or rosros.rclify.executors.Executor
        self._grp   = None  # rclpy.callback_groups.CallbackGroup or rosros.rclify.callback_groups.CallbackGroup

        self._running = False  # Whether test has been set up and is executing

        rclpy.init()
        self._node = rclpy.create_node(self.NAME, namespace=self.NAMESPACE)
        self._grp  = rclpy.callback_groups.ReentrantCallbackGroup()
        self._exec = rclpy.executors.MultiThreadedExecutor()
        self._exec.add_node(self._node)
        self.add_logging()


    def setUp(self):
        """Opens topics and services, spins ROS."""
        super().setUp()
        self.run_test_node()

        for category, opts in ((c, v) for c in ("subscribe", "service")
                               for v in testnode.DEFAULTS.get(c, {}).values()):
            if "action" not in opts:
                continue  # for category, opts

            name, typename, value = opts["name"], opts["type"], opts.get("value")
            cls = rosros.api.get_message_class(typename)
            self._types[name] = cls
            if "subscribe" == category:
                logger.info("Opening publisher to %r as %s.", name, typename)
                self._pubs[name] = self._node.create_publisher(cls, name, 2,
                                                               callback_group=self._grp)
            elif "service" == category:
                logger.info("Opening service client to %r as %s.", name, typename)
                self._clis[name] = self._node.create_client(cls, name, callback_group=self._grp)
                self._exps[name] = {} if value is None else value

            action = opts["action"]
            aname, atypename, avalue = action["name"], action["type"], action.get("value")
            acls = rosros.api.get_message_class(atypename)
            self._types[aname] = acls
            if "publish" == action["category"]:
                logger.info("Opening subscriber to %r as %s.", aname, atypename)
                handler = functools.partial(self.on_message, aname)
                self._subs[aname] = self._node.create_subscription(acls, aname, handler, 2,
                                                                   callback_group=self._grp)

            elif "service" == action["category"]:
                logger.info("Opening service server at %r as %s.", aname, atypename)
                handler = functools.partial(self.on_service, aname)
                self._srvs[aname] = self._node.create_service(acls, aname, handler,
                                                              callback_group=self._grp)
                self._exps[aname] = {} if avalue is None else avalue

            self._exps[aname] = {} if avalue is None else avalue

        self._running = True
        threading.Thread(target=self.spin).start()


    def tearDown(self):
        """Shuts down ROS."""
        logger.debug("Tearing down test.")
        self._running = False
        for k in list(self._pubs): self._node.destroy_publisher   (self._pubs.pop(k))
        for k in list(self._subs): self._node.destroy_subscription(self._subs.pop(k))
        for k in list(self._srvs): self._node.destroy_service     (self._srvs.pop(k))
        for k in list(self._clis): self._node.destroy_client      (self._clis.pop(k))
        self.shutdown_test_node()
        self._node.destroy_node()


    def spin(self):
        """Spins rclpy until shutdown."""
        while rclpy.ok() and self._running:
            rclpy.spin_once(self._node, timeout_sec=1)


    def on_message(self, name, msg):
        """Handler for incoming message, registers message object."""
        raw, loggable = ("raw ", "AnyMsg") if "*" == rosros.api.get_message_type(msg) else ("", msg)
        logger.info("Received %smessage in %r: %s.", raw, name, loggable)
        self._msgs.setdefault(name, []).append(msg)


    def on_service(self, name, req, res):
        """Handler for service call, registers request object."""
        logger.info("Incoming service call to %r with %s.", name, req)
        self._reqs.setdefault(name, []).append(req)
        return res


    def test_rclify(self):
        """Tests rclify API."""
        logger.debug("Starting test.")
        self.assertTrue(self._pubs or self._srvs, "No publishers or services configured.")
        resps = {}  # {service name: received response objects}

        logger.info("Waiting for subscribers in %s topics.", len(self._pubs))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline \
        and sum(bool(x.get_subscription_count()) for x in self._pubs.values()) < len(self._pubs):
            time.sleep(0.5)

        logger.info("Waiting for %s services.", len(self._clis))
        for name, cli in self._clis.items():
            logger.info("Waiting for service %r.", name)
            cli.wait_for_service(5)

        for name, pub in self._pubs.items():
            logger.info("Publishing to %r.", name)
            msg = self._types[name]()
            pub.publish(msg)

        for name, cli in self._clis.items():
            logger.info("Invoking service at %r.", name)
            req = rosros.api.get_service_request_class(self._types[name])()
            future = cli.call_async(req)
            self._exec.spin_until_future_complete(future)
            resps.setdefault(name, []).append(future.result())


        with self.subTest("rclpy"):               self.verify_rclpy_index()
        with self.subTest("callback_groups"):     self.verify_callback_groups()
        with self.subTest("client"):              self.verify_client()
        with self.subTest("clock"):               self.verify_clock()
        with self.subTest("duration"):            self.verify_duration()
        with self.subTest("executors"):           self.verify_executors()
        with self.subTest("node"):                self.verify_node()
        with self.subTest("parameter"):           self.verify_parameter()
        with self.subTest("publisher"):           self.verify_publisher()
        with self.subTest("qos"):                 self.verify_qos()
        with self.subTest("service"):             self.verify_service()
        with self.subTest("subscriber"):          self.verify_subscription()
        with self.subTest("task"):                self.verify_task()
        with self.subTest("time"):                self.verify_time()
        with self.subTest("timer_rate"):          self.verify_timer_rate()
        with self.subTest("topic_endpoint_info"): self.verify_topic_endpoint_info()


        logger.info("Waiting for messages in %s topics.", len(self._subs))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline \
        and sum(bool(x) for x in self._msgs.values()) < len(self._subs):
            time.sleep(1)

        logger.info("Verifying received messages in %s topics.", len(self._subs))
        for name in self._subs:
            logger.info("Verifying received messages in %r.", name)
            with self.subTest(name):
                self.assertTrue(self._msgs.get(name),
                                "Expected message not received in topic %r." % name)
                for k, v in self._exps[name].items():
                    if isinstance(v, (dict, list)): continue  # for k, v
                    logger.info("Verifying %s=%r in %r.", k, v, name)
                    self.assertEqual(rosros.util.get_value(self._msgs[name][0], k), v,
                                     "Unexpected value in %s." % k)

        logger.info("Waiting for actions in %s service clients.", len(self._clis))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline \
        and not all(self._reqs.get(name) for name in self._srvs):
            time.sleep(1)

        logger.info("Verifying received responses for %s service clients.", len(self._clis))
        for name in self._clis:
            logger.info("Verifying received responses for service %r.", name)
            with self.subTest(name):
                self.assertTrue(resps.get(name),
                                "Expected response not received in service %r." % name)
                for k, v in self._exps[name].items():
                    if isinstance(v, (dict, list)): continue  # for k, v
                    logger.info("Verifying %s=%r in %r.", k, v, name)
                    self.assertEqual(rosros.util.get_value(resps[name][0], k), v,
                                     "Unexpected value in %s." % k)

        logger.info("Verifying received requests for %s services.", len(self._srvs))
        for name in self._srvs:
            logger.info("Verifying received requests in %r.", name)
            with self.subTest(name):
                self.assertTrue(self._reqs.get(name),
                                "Expected request not received in service %r." % name)
                for k, v in self._exps[name].items():
                    if isinstance(v, (dict, list)): continue  # for k, v
                    logger.info("Verifying %s=%r in %r.", k, v, name)
                    self.assertEqual(rosros.util.get_value(self._reqs[name][0], k), v,
                                     "Unexpected value in %s." % k)


    def verify_callback_groups(self):
        """Tests rclify CallbackGroup API."""
        logger.info("Testing CallbackGroup API.")

        for name in ("CallbackGroup", "MutuallyExclusiveCallbackGroup", "ReentrantCallbackGroup"):
            cls = getattr(rclpy.callback_groups, name, None)
            self.assertTrue(issubclass(cls, rclpy.callback_groups.CallbackGroup),
                            "Unexpected type for rclpy.callback_groups.%s (%r)." % (name, cls))
            instance = cls()
            for attrname in ("add_entity", "beginning_execution", "can_execute",
                             "ending_execution", "has_entity"):
                attr = getattr(instance, attrname, None)
                self.assertIsNotNone(attr, "Unexpected value for rclpy.callback_groups.%s.%s (%r)" %
                                           (name, attrname, attr))
            instance.add_entity(self)
            self.assertTrue(instance.has_entity(self),
                            "Unexpected value from rclpy.callback_groups.%s.has_entity()." % name)


    def verify_client(self):
        """Tests rclify ServiceProxy API."""
        logger.info("Testing Client API.")
        for cli in self._clis.values():
            self.assertTrue(callable(getattr(cli, "call_async")),
                            "Unexpected value for Service.call_async.")
            self.assertTrue(callable(getattr(cli, "destroy")),
                            "Unexpected value for Service.destroy.")
            self.assertTrue(callable(getattr(cli, "remove_pending_request")),
                            "Unexpected value for Service.remove_pending_request.")
            self.assertTrue(callable(getattr(cli, "service_is_ready")),
                            "Unexpected value for Service.service_is_ready.")
            self.assertTrue(callable(getattr(cli, "wait_for_service")),
                            "Unexpected value for Service.wait_for_service.")
            self.assertTrue(cli.service_is_ready(), "Unexpected value for Service.service_is_ready().")


    def verify_clock(self):
        """Verifies rclify Clock API."""
        logger.info("Testing Clock API.")

        for name in ("ROS_TIME", "STEADY_TIME", "SYSTEM_TIME"):
            logger.info("Testing ClockType.%s.", name)
            clock_type = getattr(rclpy.clock.ClockType, name, None)
            self.assertIsNotNone(clock_type, "Unexpected value for ClockType.%s." % name)
            clock = rclpy.clock.Clock(clock_type=clock_type)
            t = clock.now()
            self.assertIsInstance(t, rclpy.time.Time)
            if "ROS_TIME" == name:
                expected = self._node.get_clock().now().nanoseconds
            elif "STEADY_TIME" == name:
                expected = time.monotonic_ns()
            else:
                expected = time.time_ns()
            self.assertAlmostEqual(t.nanoseconds, expected, delta=1E11,
                                   msg="Unexpected value for Clock(%s).now()." % name)


    def verify_duration(self):
        """Verifies rclify Duration API."""
        logger.info("Testing Duration API.")

        dur1 = rclpy.duration.Duration(seconds=10, nanoseconds=20000000)
        self.assertEqual(dur1.nanoseconds, 10020000000, "Unexpected value for Duration.nanoseconds.")

        durmsg = dur1.to_msg()
        self.assertIsNotNone(durmsg, "Unexpected value for Duration.to_msg().")
        dur2 = rclpy.duration.Duration.from_msg(durmsg)
        self.assertTrue (dur2,       "Unexpected value for Duration.from_msg().")
        self.assertEqual(dur2, dur1, "Unexpected value for Duration.from_msg().")


    def verify_time(self):
        """Verifies rclify Time API."""
        logger.info("Testing Time API.")
        tm0 = rclpy.time.Time(seconds=10, nanoseconds=20000000)
        tm1 = rclpy.time.Time(seconds=10, nanoseconds=20000000, clock_type=rclpy.clock.ClockType.STEADY_TIME)
        self.assertEqual(tm0.nanoseconds, 10020000000, "Unexpected value for Time.nanoseconds.")
        self.assertEqual(tm0.clock_type, rclpy.clock.ClockType.SYSTEM_TIME,
                         "Unexpected value for Time.clock_type.")
        self.assertEqual(tm1.clock_type, rclpy.clock.ClockType.STEADY_TIME,
                         "Unexpected value for Time.clock_type.")
        self.assertEqual(tm0.seconds_nanoseconds(), (10, 20000000),
                         "Unexpected value for Time.seconds_nanoseconds().")

        tmsg = tm0.to_msg()
        self.assertTrue(tmsg, "Unexpected value for Time.to_msg().")
        tm2 = rclpy.time.Time.from_msg(tmsg, clock_type=tm0.clock_type)
        self.assertTrue(tm2, "Unexpected value for Time.from_msg().")
        self.assertEqual(tm2, tm0, "Unexpected value for Time.from_msg().")


    def verify_executors(self):
        """Tests rclify Executor API."""
        logger.info("Testing Executor API.")

        for name in ("ConditionReachedException", "ExternalShutdownException",
                     "ShutdownException", "TimeoutException"):
            cls = getattr(rclpy.executors, name, None)
            self.assertTrue(issubclass(cls, Exception),
                            "Unexpected type for rclpy.executors.%s (%r)." % (name, cls))

        for name in ("Executor", "MultiThreadedExecutor", "SingleThreadedExecutor"):
            cls = getattr(rclpy.executors, name, None)
            self.assertTrue(issubclass(cls, rclpy.executors.Executor),
                            "Unexpected type for rclpy.executors.%s (%r)." % (name, cls))
            instance = cls(context=self._node.context)
            for attrname in ("add_node", "context", "create_task", "get_nodes", "remove_node",
                             "shutdown", "spin", "spin_once", "spin_once_until_future_complete",
                             "spin_until_future_complete", "wake"):
                exists = hasattr(instance, attrname)
                self.assertTrue(exists, "Unexpected value for rclpy.executors.%s.%s." %
                                        (name, attrname))
            self.assertEqual(instance.context, self._node.context,
                             "Unexpected value for rclpy.node.Node.context.")
            instance.shutdown()


    def verify_node(self):
        """Tests rclify Node API."""
        logger.info("Testing Node API.")

        self.assertTrue(issubclass(getattr(rclpy.node, "NodeNameNonExistentError", None),
                                   Exception),
                        "Unexpected value for rclpy.node.NodeNameNonExistentError.")

        self.assertTrue(list(self._node.publishers),
                        "Unexpected value for rclpy.node.Node.publishers.")
        self.assertTrue(list(self._node.subscriptions),
                        "Unexpected value for rclpy.node.Node.subscriptions.")
        self.assertTrue(list(self._node.clients),
                        "Unexpected value for rclpy.node.Node.clients.")
        self.assertTrue(list(self._node.services),
                        "Unexpected value for rclpy.node.Node.services.")
        self.assertTrue(inspect.isgenerator(self._node.timers),
                        "Unexpected value for rclpy.node.Node.timers.")
        self.assertTrue(inspect.isgenerator(self._node.guards),
                        "Unexpected value for rclpy.node.Node.guards.")
        self.assertTrue(inspect.isgenerator(self._node.waitables),
                        "Unexpected value for rclpy.node.Node.waitables.")

        self.assertEqual(self._node.get_name(), self.NAME,
                         "Unexpected value for rclpy.node.Node.get_name()")
        self.assertEqual(self._node.get_namespace(), self.NAMESPACE,
                         "Unexpected value for rclpy.node.Node.get_namespace()")
        self.assertIsInstance(self._node.get_clock(), rclpy.clock.Clock,
                         "Unexpected value for rclpy.node.Node.get_clock()")
        self.assertIsNotNone(self._node.get_logger(),
                         "Unexpected value for rclpy.node.Node.get_logger()")

        def make_callback(lst, resulter):
            """
            Returns function that calls resulter() with parameters,
            adds result to list and returns SetParametersResult.
            """
            def inner(params):
                success = resulter(params)
                lst.append(success)
                return rclpy.node.SetParametersResult(successful=success)
            return inner

        cbs1, cbs2, cbs3, cbs_empty = [], [], [], []

        self._node.set_parameters_callback(make_callback(cbs1, lambda pp: True))
        self._node.add_on_set_parameters_callback(make_callback(cbs2, lambda p: True))

        removed_cb = make_callback(cbs_empty, lambda pp: False)
        self._node.add_on_set_parameters_callback(removed_cb)
        self._node.remove_on_set_parameters_callback(removed_cb)


        param = self._node.get_parameter_or("nope")
        self.assertIsInstance(param, rclpy.node.Parameter,
                              "Unexpected value for rclpy.node.Node.get_parameter_or().")
        self.assertEqual(param.type_, rclpy.node.Parameter.Type.NOT_SET,
                              "Unexpected value for rclpy.node.Node.get_parameter_or().")
        self.assertEqual(param.value, None,
                              "Unexpected value for rclpy.node.Node.get_parameter_or().")

        param = self._node.declare_parameter("declare.param.v1", 5,
            rclpy.node.ParameterDescriptor(name="declare.param.v1",
                                           type=rclpy.node.Parameter.Type.INTEGER.value,
                                           description="v1 description"))
        self.assertIsInstance(param, rclpy.node.Parameter,
                              "Unexpected value for rclpy.node.Node.declare_parameter().")
        self.assertTrue(self._node.has_parameter("declare.param.v1"),
                        "Unexpected state from rclpy.node.Node.declare_parameter().")
        self.assertEqual(self._node.get_parameter("declare.param.v1"), param,
                         "Unexpected value for rclpy.node.Node.get_parameter().")
        self.assertEqual(cbs1, [True],
                         "Unexpected state from rclpy.node.Node.declare_parameter().")
        self.assertEqual(cbs2, [True],
                         "Unexpected state from rclpy.node.Node.declare_parameter().")
        self.assertFalse(cbs_empty,
                         "Unexpected state from rclpy.node.Node.declare_parameter().")
        cbs1.clear(), cbs2.clear()


        params = self._node.declare_parameters("declare", [("params.v1", 1),
                                                           ("params.v2", 2)])
        self.assertEqual(len(params), 2, "Unexpected value from rclpy.node.set_parameters().")
        self.assertIsInstance(params[0], rclpy.node.Parameter,
                              "Unexpected value for rclpy.node.Node.declare_parameters().")
        self.assertEqual(params[0].name, "declare.params.v1",
                         "Unexpected value for rclpy.node.Node.declare_parameters().")
        self.assertEqual(params[1].value, 2,
                         "Unexpected value for rclpy.node.Node.declare_parameters().")
        self.assertEqual(cbs1, [True, True],
                         "Unexpected state from rclpy.node.Node.declare_parameters().")

        params[0] = rclpy.node.Parameter(name=params[0].name, type_=params[0].type_, value=11)
        params[1] = rclpy.node.Parameter(name=params[1].name, type_=params[1].type_, value=22)
        cbs1.clear(), cbs2.clear()
        results = self._node.set_parameters(params)
        self.assertEqual(len(results), 2, "Unexpected value from rclpy.node.set_parameters().")
        self.assertTrue(all(x.successful for x in results),
                         "Unexpected value for rclpy.node.Node.set_parameters().")
        self.assertEqual(cbs2, [True, True],
                         "Unexpected state from rclpy.node.Node.set_parameters().")

        params = self._node.get_parameters_by_prefix("declare")
        self.assertEqual(len(params), 3, "Unexpected value from rclpy.node.get_parameters_by_prefix().")
        self.assertEqual(params["param.v1"].name, "declare.param.v1",
                         "Unexpected value for rclpy.node.Node.get_parameters_by_prefix().")

        params = self._node.get_parameters_by_prefix("declare.params")
        self.assertEqual(len(params), 2, "Unexpected value from rclpy.node.get_parameters_by_prefix().")
        self.assertEqual(params["v2"].name, "declare.params.v2",
                         "Unexpected value for rclpy.node.Node.get_parameters_by_prefix().")

        desc = self._node.describe_parameter("declare.param.v1")
        self.assertIsInstance(desc, rclpy.node.ParameterDescriptor,
                              "Unexpected state from rclpy.node.Node.describe_parameter().")
        self.assertEqual(desc.type, rclpy.node.Parameter.Type.INTEGER.value,
                         "Unexpected value for rclpy.node.Node.describe_parameter().")
        self.assertEqual(desc.description, "v1 description",
                         "Unexpected value for rclpy.node.Node.describe_parameter().")

        with self.assertRaises(rclpy.exceptions.InvalidParameterValueException,
                               msg="Unexpected success for rclpy.node.Node.set_descriptor()."):
            self._node.set_descriptor("declare.param.v1",
                rclpy.node.ParameterDescriptor(
                    type=rclpy.node.Parameter.Type.INTEGER.value,
                    integer_range=[rclpy.node.IntegerRange(from_value=10, to_value=20)]
                ),
            )

        value = self._node.set_descriptor("declare.param.v1",
            rclpy.node.ParameterDescriptor(type=rclpy.node.Parameter.Type.INTEGER.value,
                                           description="x",
                                           integer_range=[rclpy.node.IntegerRange(from_value=10,
                                                                                  to_value=20)]),
            rclpy.node.ParameterValue(type=rclpy.node.Parameter.Type.INTEGER.value,
                                      integer_value=15)
        )
        self.assertEqual(value.integer_value, 15,
                        "Unexpected value for rclpy.node.Node.set_descriptor().")
        self.assertEqual(self._node.get_parameter("declare.param.v1").value, 15,
                        "Unexpected state from rclpy.node.Node.set_descriptor().")

        descs = self._node.describe_parameters(["declare.param.v1", "declare.params.v2"])
        self.assertEqual(len(descs), 2,
                         "Unexpected value for rclpy.node.Node.describe_parameters().")
        self.assertEqual(descs[0].type, rclpy.node.Parameter.Type.INTEGER.value,
                         "Unexpected state from rclpy.node.Node.set_descriptor().")
        self.assertEqual(descs[0].description, "x",
                         "Unexpected state from rclpy.node.Node.set_descriptor().")
        self.assertEqual(descs[1].name, "declare.params.v2",
                         "Unexpected value for rclpy.node.Node.describe_parameters().")

        self._node.undeclare_parameter("declare.param.v1")
        self.assertFalse(self._node.has_parameter("declare.param.v1"),
                        "Unexpected state from rclpy.node.Node.undeclare_parameter().")

        params = self._node.get_parameters(["declare.params.v1", "declare.params.v2"])
        self.assertEqual(len(params), 2,
                         "Unexpected value for rclpy.node.Node.get_parameters().")
        self.assertIsInstance(params[0], rclpy.node.Parameter,
                         "Unexpected value for rclpy.node.Node.get_parameters().")
        self.assertEqual(params[0].get_parameter_value().integer_value, 11,
                         "Unexpected value for rclpy.node.Node.get_parameters().")

        with self.assertRaises(rclpy.exceptions.ParameterNotDeclaredException,
                               msg="Unexpected success for rclpy.node.Node.set_parameters()."):
            self._node.set_parameters([rclpy.node.Parameter("rejected0")])
        self.assertFalse(self._node.has_parameter("rejected0"),
                         "Unexpected state from rclpy.node.Node.set_parameters().")

        # Verify that a rejecting callback gets enforced
        self._node.declare_parameter("rejected1", "")
        self._node.declare_parameter("rejected2", "")
        self._node.add_on_set_parameters_callback(make_callback(cbs3, lambda pp: False))
        results = self._node.set_parameters([rclpy.node.Parameter("rejected1")])
        self.assertTrue(results, "Unexpected value for rclpy.node.Node.set_parameters().")

        result = self._node.set_parameters_atomically([rclpy.node.Parameter("rejected1"),
                                                       rclpy.node.Parameter("rejected2")])
        self.assertIsInstance(result, rclpy.node.SetParametersResult,
                         "Unexpected value for rclpy.node.Node.set_parameters_atomically().")
        self.assertFalse(results[0].successful,
                         "Unexpected value for rclpy.node.Node.set_parameters_atomically().")

        self.assertFalse(cbs_empty, "Unexpected invocation of a removed set-parameters-callback.")


        for name in ("add_waitable", "remove_waitable", "create_guard_condition",
                     "destroy_client", "destroy_guard_condition", "destroy_node", "destroy_publisher",
                     "destroy_rate", "destroy_service", "destroy_subscription", "destroy_timer"):
            self.assertTrue(callable(getattr(self._node, name, None)),
                            "Unexpected value for rclpy.node.Node.%s." % name)

        self.assertGreater(self._node.count_publishers(next(iter(self._subs))), 0,
                           "Unexpected value for rclpy.node.Node.count_publishers().")
        for topic in self._pubs:
            self.assertGreater(self._node.count_subscribers(topic), 0,
                               "Unexpected value for rclpy.node.Node.count_subscribers().")

        self.assertIsInstance(self._node.get_client_names_and_types_by_node(self.NAME, self.NAMESPACE),
                              list, "Unexpected value for "
                                    "rclpy.node.Node.get_client_names_and_types_by_node().")

        self.assertIn(testnode.TestNode.NAME, self._node.get_node_names(),
                      "Unexpected value for rclpy.node.Node.get_node_names().")
        self.assertIn((testnode.TestNode.NAME, self.NAMESPACE),
                      self._node.get_node_names_and_namespaces(),
                      "Unexpected value for rclpy.node.Node.get_node_names_and_namespaces().")
        self.assertIn((testnode.TestNode.NAME, self.NAMESPACE, "/"),
                      self._node.get_node_names_and_namespaces_with_enclaves(),
                      "Unexpected value for "
                      "rclpy.node.Node.get_node_names_and_namespaces_with_enclaves().")

        pubs = self._node.get_publisher_names_and_types_by_node(testnode.TestNode.NAME,
                                                                self.NAMESPACE)
        self.assertIsInstance(pubs, list,
                              "Unexpected value for "
                              "rclpy.node.Node.get_publisher_names_and_types_by_node().")
        self.assertIsInstance(pubs[0], tuple,
                              "Unexpected value for "
                              "rclpy.node.Node.get_publisher_names_and_types_by_node().")
        self.assertIsInstance(pubs[0][0], str,
                              "Unexpected value for "
                              "rclpy.node.Node.get_publisher_names_and_types_by_node().")
        self.assertIsInstance(pubs[0][1], list,
                              "Unexpected value for "
                              "rclpy.node.Node.get_publisher_names_and_types_by_node().")
        self.assertIsInstance(pubs[0][1][0], str,
                              "Unexpected value for "
                              "rclpy.node.Node.get_publisher_names_and_types_by_node().")

        srvs = self._node.get_service_names_and_types()
        self.assertIsInstance(srvs, list,
                              "Unexpected value for rclpy.node.Node.get_service_names_and_types().")

        srvs = self._node.get_service_names_and_types_by_node(testnode.TestNode.NAME,
                                                              self.NAMESPACE)
        self.assertIsInstance(srvs, list,
                              "Unexpected value for "
                              "rclpy.node.Node.get_service_names_and_types_by_node().")
        self.assertTrue(set(self._clis) & set(x[0] for x in srvs),
                            "Unexpected value for "
                            "rclpy.node.Node.get_service_names_and_types_by_node().")

        subs = self._node.get_subscriber_names_and_types_by_node(testnode.TestNode.NAME,
                                                                 self.NAMESPACE)
        self.assertIsInstance(subs, list,
                              "Unexpected value for "
                              "rclpy.node.Node.get_subscriber_names_and_types_by_node().")
        self.assertTrue(set(self._pubs) & set(x[0] for x in subs),
                            "Unexpected value for "
                            "rclpy.node.Node.get_subscriber_names_and_types_by_node().")

        topics = self._node.get_topic_names_and_types()
        self.assertIsInstance(topics, list,
                              "Unexpected value for "
                              "rclpy.node.Node.get_topic_names_and_types().")
        self.assertTrue(set(self._subs) & set(x[0] for x in topics),
                            "Unexpected value for "
                            "rclpy.node.Node.get_topic_names_and_types().")

        infos = self._node.get_publishers_info_by_topic(next(iter(self._subs)))
        self.assertTrue(infos,
                        "Unexpected value for rclpy.node.Node.get_publishers_info_by_topic().")
        self.assertIsInstance(infos[0], rclpy.topic_endpoint_info.TopicEndpointInfo,
                        "Unexpected value for rclpy.node.Node.get_publishers_info_by_topic().")

        infos = self._node.get_subscriptions_info_by_topic(next(iter(self._pubs)))
        self.assertTrue(infos,
                        "Unexpected value for rclpy.node.Node.get_subscriptions_info_by_topic().")
        self.assertIsInstance(infos[0], rclpy.topic_endpoint_info.TopicEndpointInfo,
                        "Unexpected value for rclpy.node.Node.get_subscriptions_info_by_topic().")


    def verify_parameter(self):
        """Tests rclify Parameter API."""
        logger.info("Testing Parameter API.")
        self.assertIsInstance(getattr(rclpy.parameter, "PARAMETER_SEPARATOR_STRING", None), str,
                              "Unexpected value for rclpy.parameter.PARAMETER_SEPARATOR_STRING.")

        VALS = (None, True, 3, 1.2, "str", [b"b", b"y"], [True, False], [1, 2], [1.2, 3.4], ["str"])
        for param_type, value in zip(rclpy.parameter.Parameter.Type, VALS):
            p = rclpy.parameter.Parameter(param_type.name, param_type, value)
            v = p.get_parameter_value()
            self.assertEqual(v.type, param_type.value,
                             "Unexpected value for Parameter.get_parameter_value().")
        with self.assertRaises(Exception, msg="Unexpected success with wrong type in Parameter()."):
            rclpy.parameter.Parameter("name", rclpy.parameter.Parameter.Type.Bool, 2)
        self.assertTrue(hasattr(rclpy.parameter, "Parameter"),
                        "Unexpected value for rclpy.parameter.Parameter.")
        self.assertTrue(hasattr(rclpy.parameter.Parameter, "Type"),
                        "Unexpected value for rclpy.parameter.Parameter.Type.")


    def verify_publisher(self):
        """Tests rclify Publisher API."""
        logger.info("Testing Publisher API.")
        for name, pub in self._pubs.items():
            self.assertEqual(pub.topic_name, name,
                             "Unexpected value for Publisher.topic_name.")
            self.assertIsNone(pub.assert_liveliness(),
                              "Unexpected value for Publisher.assert_liveliness().")
            self.assertIsInstance(pub.get_subscription_count(), int,
                                  "Unexpected value for Publisher.get_subscription_count().")
            self.assertTrue(callable(getattr(pub, "destroy")),
                            "Unexpected value for Publisher.destroy.")


    def verify_qos(self):
        """Tests rclify QoS API."""
        logger.info("Testing QoS API.")

        POLICY_KINDS      = ["INVALID", "DURABILITY", "DEADLINE", "LIVELINESS",
                             "RELIABILITY", "HISTORY", "LIFESPAN"]
        HISTORY_KINDS     = ["SYSTEM_DEFAULT", "KEEP_LAST", "KEEP_ALL", "UNKNOWN"]
        RELIABILITY_KINDS = ["SYSTEM_DEFAULT", "RELIABLE", "BEST_EFFORT", "UNKNOWN"]
        DURABILITY_KINDS  = ["SYSTEM_DEFAULT", "TRANSIENT_LOCAL", "VOLATILE", "UNKNOWN"]
        LIVELINESS_KINDS  = ["SYSTEM_DEFAULT", "AUTOMATIC", "MANUAL_BY_TOPIC", "UNKNOWN"]
        PROFILES          = ["UNKNOWN", "SYSTEM_DEFAULT", "SENSOR_DATA", "SERVICES_DEFAULT",
                             "PARAMETERS", "PARAMETER_EVENTS", "ACTION_STATUS_DEFAULT"]

        self.assertIsNotNone(getattr(rclpy.qos, "QoSPolicyKind", None),
                             "Unexpected value for rclpy.qos.QoSPolicyKind.")
        for name in POLICY_KINDS:
            self.assertIsNotNone(getattr(rclpy.qos.QoSPolicyKind, name, None),
                                 "Unexpected value for rclpy.qos.QoSPolicyKind.%s." % name)

        self.assertIsNotNone(getattr(rclpy.qos, "HistoryPolicy", None),
                             "Unexpected value for rclpy.qos.HistoryPolicy.")
        self.assertIsNotNone(getattr(rclpy.qos, "QoSHistoryPolicy", None),
                             "Unexpected value for rclpy.qos.QoSHistoryPolicy.")
        for name in HISTORY_KINDS:
            self.assertIsNotNone(getattr(rclpy.qos.HistoryPolicy, name, None),
                                 "Unexpected value for rclpy.qos.HistoryPolicy.%s." % name)

        self.assertIsNotNone(getattr(rclpy.qos, "ReliabilityPolicy", None),
                             "Unexpected value for rclpy.qos.ReliabilityPolicy.")
        self.assertIsNotNone(getattr(rclpy.qos, "QoSReliabilityPolicy", None),
                             "Unexpected value for rclpy.qos.QoSReliabilityPolicy.")
        for name in RELIABILITY_KINDS:
            self.assertIsNotNone(getattr(rclpy.qos.ReliabilityPolicy, name, None),
                                 "Unexpected value for rclpy.qos.ReliabilityPolicy.%s." % name)

        self.assertIsNotNone(getattr(rclpy.qos, "DurabilityPolicy", None),
                             "Unexpected value for rclpy.qos.DurabilityPolicy.")
        self.assertIsNotNone(getattr(rclpy.qos, "QoSDurabilityPolicy", None),
                             "Unexpected value for rclpy.qos.QoSDurabilityPolicy.")
        for name in DURABILITY_KINDS:
            self.assertIsNotNone(getattr(rclpy.qos.DurabilityPolicy, name, None),
                                 "Unexpected value for rclpy.qos.DurabilityPolicy.%s." % name)

        self.assertIsNotNone(getattr(rclpy.qos, "LivelinessPolicy", None),
                             "Unexpected value for rclpy.qos.LivelinessPolicy.")
        self.assertIsNotNone(getattr(rclpy.qos, "QoSLivelinessPolicy", None),
                             "Unexpected value for rclpy.qos.QoSLivelinessPolicy.")
        for name in LIVELINESS_KINDS:
            self.assertIsNotNone(getattr(rclpy.qos.LivelinessPolicy, name, None),
                                 "Unexpected value for rclpy.qos.LivelinessPolicy.%s." % name)

        self.assertTrue(issubclass(getattr(rclpy.qos, "InvalidQoSProfileException", None),
                                   Exception),
                              "Unexpected value for rclpy.qos.InvalidQoSProfileException.")
        self.assertIsNotNone(getattr(rclpy.qos, "QoSPresetProfiles", None),
                             "Unexpected value for rclpy.qos.QoSPresetProfiles.")
        for name in PROFILES:
            self.assertIsNotNone(getattr(rclpy.qos.QoSPresetProfiles, name, None),
                                 "Unexpected value for rclpy.qos.QoSPresetProfiles.%s." % name)

        with self.assertRaises(Exception, msg="Unexpected success with no arguments in QoSProfile()."):
            rclpy.qos.QoSProfile()

        self.assertIsNotNone(rclpy.qos.QoSProfile(depth=2),
                             "Unexpected value for rclpy.qos.QoSProfile().")


    def verify_service(self):
        """Tests rclify Service API."""
        logger.info("Testing Service API.")
        for srv in self._srvs.values():
            self.assertTrue(callable(getattr(srv, "destroy")),
                            "Unexpected value for Service.destroy.")


    def verify_subscription(self):
        """Tests rclify Subscription API."""
        logger.info("Testing Subscription API.")
        for name, sub in self._subs.items():
            self.assertEqual(sub.topic_name, name, "Unexpected value for Subscription.topic_name.")
            self.assertTrue(callable(getattr(sub, "destroy")),
                            "Unexpected value for Subscription.destroy.")


    def verify_task(self):
        """Tests rclify Task API."""
        logger.info("Testing Task API.")

        calls = []
        future = rclpy.task.Future()
        future.add_done_callback(calls.append)
        self.assertFalse(future.done(),       "Unexpected value for rclpy.task.Future.done().")
        self.assertFalse(future.cancelled(),  "Unexpected value for rclpy.task.Future.cancelled().")
        self.assertIsNone(future.result(),    "Unexpected value for rclpy.task.Future.result().")
        self.assertIsNone(future.exception(), "Unexpected value for rclpy.task.Future.exception().")
        future.set_result(self)
        self.assertTrue(future.done(),          "Unexpected value for rclpy.task.Future.done().")
        self.assertEqual(future.result(), self, "Unexpected value for rclpy.task.Future.result().")
        self.assertTrue(calls,                  "Done-callbacks not invoked for rclpy.task.Future.")

        calls.clear()
        task = rclpy.task.Task(lambda: calls.append(self) or self)
        self.assertFalse(task.done(),       "Unexpected value for rclpy.task.Task.done().")
        self.assertFalse(task.cancelled(),  "Unexpected value for rclpy.task.Task.cancelled().")
        self.assertIsNone(task.result(),    "Unexpected value for rclpy.task.Task.result().")
        self.assertIsNone(task.exception(), "Unexpected value for rclpy.task.Task.exception().")
        self.assertFalse(task.executing(), "Unexpected value for rclpy.task.Task.executing().")
        task()
        self.assertTrue(task.done(),          "Unexpected value for rclpy.task.Task.done().")
        self.assertEqual(task.result(), self, "Unexpected value for rclpy.task.Task.result().")
        self.assertTrue(calls, "Handler not invoked for rclpy.task.Task.")


    def verify_timer_rate(self):
        """Verifies rclify Timer and Rate API."""
        logger.info("Testing Timer and Rate API.")
        calls = []
        t = self._node.create_timer(1, lambda: calls.append(None), callback_group=self._grp)
        r = self._node.create_rate(0.5)
        r.sleep()

        self.assertTrue(callable(getattr(r, "destroy")),
                        "Unexpected value for Rate.destroy().")

        self.assertTrue(callable(getattr(t, "destroy")),
                        "Unexpected value for Timer.destroy().")
        self.assertFalse(t.is_canceled(),  "Unexpected value for Timer.is_canceled().")
        self.assertIsInstance(t.is_ready(), bool, "Unexpected value for Timer.is_ready().")
        self.assertIsNotNone(t.clock, "Unexpected value for Timer.clock.")
        self.assertEqual(t.clock.clock_type, rclpy.clock.ClockType.ROS_TIME,
                        "Unexpected value for Timer.clock.clock_type.")
        self.assertEqual(t.timer_period_ns, 10**9, "Unexpected value for Timer.timer_period_ns.")
        self.assertIsInstance(t.time_since_last_call(), int,
                              "Unexpected value for Timer.time_since_last_call().")
        self.assertIsInstance(t.time_until_next_call(), int,
                              "Unexpected value for Timer.time_until_next_call().")

        t.cancel()
        r.sleep()
        self.assertTrue(t.is_canceled(), "Unexpected value for Timer.is_canceled().")
        t.destroy()
        r.destroy()


    def verify_topic_endpoint_info(self):
        """Tests rclify TopicEndpointInfo API."""
        logger.info("Testing TopicEndpointInfo API.")

        enum = getattr(rclpy.topic_endpoint_info, "TopicEndpointTypeEnum", None)
        cls  = getattr(rclpy.topic_endpoint_info, "TopicEndpointInfo",     None)

        self.assertIsNotNone(enum, "Unexpected value for "
                             "rclpy.topic_endpoint_info.TopicEndpointTypeEnum.")
        self.assertIsNotNone(cls, "Unexpected value for "
                             "rclpy.topic_endpoint_info.TopicEndpointInfo.")

        for name in ("INVALID", "PUBLISHER", "SUBSCRIPTION"):
            self.assertIsNotNone(getattr(enum, name, None), "Unexpected value for "
                                 "rclpy.topic_endpoint_info.TopicEndpointTypeEnum.%s" % name)

        instance = cls()
        instance.node_name      = "node_name"
        instance.node_namespace = "node_namespace"
        instance.topic_type     = "topic_type"
        instance.endpoint_type  = enum.PUBLISHER
        instance.endpoint_gid   = list(range(5))
        instance.qos_profile    = dict(depth=5)
        self.assertEqual(instance.node_name, "node_name", "Unexpected value for "
                         "rclpy.topic_endpoint_info.TopicEndpointInfo.node_name.")
        self.assertEqual(instance.node_namespace, "node_namespace", "Unexpected value for "
                         "rclpy.topic_endpoint_info.TopicEndpointInfo.node_namespace.")
        self.assertEqual(instance.topic_type, "topic_type", "Unexpected value for "
                         "rclpy.topic_endpoint_info.TopicEndpointInfo.topic_type.")
        self.assertEqual(instance.endpoint_type, enum.PUBLISHER, "Unexpected value for "
                         "rclpy.topic_endpoint_info.TopicEndpointInfo.endpoint_type.")
        self.assertEqual(instance.endpoint_gid, list(range(5)), "Unexpected value for "
                         "rclpy.topic_endpoint_info.TopicEndpointInfo.endpoint_gid.")
        self.assertIsInstance(instance.qos_profile, rclpy.qos.QoSProfile, "Unexpected value for "
                         "rclpy.topic_endpoint_info.TopicEndpointInfo.qos_profile.")


    def verify_rclpy_index(self):
        """Tests rclify main functions."""
        logger.info("Testing rclify index API.")

        FUNCS = [
            "create_node", "get_default_context", "get_global_executor",
            "get_rmw_implementation_identifier", "init", "ok", "shutdown", "spin", "spin_once",
            "spin_until_future_complete", "try_shutdown"
        ]
        for name in FUNCS:
            self.assertIsNotNone(callable(getattr(rclpy, name, None)),
                                 "No function %r in rclpy API." % name)

        self.assertTrue(rclpy.ok(), "Unexpected value from ok().")
        self.assertIsInstance(rclpy.get_rmw_implementation_identifier(), str,
                              "Unexpected type from get_rmw_implementation_identifier().")


if "__main__" == __name__:
    TestRclify.run_rostest()
