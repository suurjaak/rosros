#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test: use rospify stand-in for rospy in ROS2.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     16.04.2022
@modified    19.04.2022
------------------------------------------------------------------------------
"""
import copy
import functools
import logging
import os
import sys
import threading
import time
import traceback

import rosros
from rosros import rospify as rospy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from test import testbase, testnode

logger = logging.getLogger()


class TestRospify(testbase.TestBase):
    """Tests using rospify stand-in for rospy in ROS2."""

    ## Test name used in flow logging
    NAME = "test_rospify"

    ## Topic remaps as {from: to}
    REMAPS = {"/foo": "/bar"}


    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._pubs = {}  # {topic name: publisher instance}
        self._subs = {}  # {topic name: subscriber instance}
        self._clis = {}  # {service name: service client instance}
        self._srvs = {}  # {service name: service server instance}
        self._msgs = {}  # {topic name: [incoming message, ]}
        self._reqs = {}  # {service name: service request instance}
        self._exps = {}  # {topic/service name: expected message data dictionary}

        self._extramsgs  = {}    # {topic name: [incoming message from added callback, ]}
        self._anymsg_sub = None  # Subscriber instance using AnyMsg
        self._anymsgs    = []    # [incoming raw message, ]

        rospy.init_node(self.NAME, ["%s:=%s" % x for x in self.REMAPS.items()])


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
            if "subscribe" == category:
                logger.info("Opening publisher to %r as %s.", name, type)
                self._pubs[name] = rospy.Publisher(name, cls)
            elif "service" == category:
                logger.info("Opening service client to %r as %s.", name, typename)
                self._clis[name] = rospy.ServiceProxy(name, cls)
                self._exps[name] = {} if value is None else value

            action = opts["action"]
            aname, atypename, avalue = action["name"], action["type"], action.get("value")
            acls = rosros.api.get_message_class(atypename)
            if "publish" == action["category"]:
                logger.info("Opening subscriber to %r as %s.", aname, atypename)
                self._subs[aname] = rospy.Subscriber(aname, acls, self.on_message, aname)
                self._subs[aname].add_callback(self.on_extra_message, aname)
                self._subs[aname].add_callback(self.on_extra_message, "NOT" + aname)
                self._subs[aname].remove_callback(self.on_extra_message, "NOT" + aname)

            elif "service" == action["category"]:
                logger.info("Opening service server at %r as %s.", aname, atypename)
                handler = functools.partial(self.on_service, aname)
                self._srvs[aname] = rospy.Service(aname, acls, handler)
                self._exps[aname] = {} if avalue is None else avalue

            self._exps[aname] = {} if avalue is None else avalue

        threading.Thread(target=rospy.spin).start()


    def tearDown(self):
        """Shuts down ROS."""
        logger.debug("Tearing down test.")
        for k in list(self._pubs): self._pubs.pop(k).unregister()
        for k in list(self._subs): self._subs.pop(k).unregister()
        for k in list(self._srvs): self._srvs.pop(k).shutdown()
        for k in list(self._clis): self._clis.pop(k).close()
        self.shutdown_test_node()
        rospy.signal_shutdown()


    def on_message(self, msg, name):
        """Handler for incoming message, registers message object."""
        logger.info("Received message in %r: %s.", name, msg)
        self._msgs.setdefault(name, []).append(msg)


    def on_extra_message(self, msg, name):
        """Handler for incoming message via added callback, registers message object."""
        logger.info("Received message via added callback in %r: %s.", name, msg)
        self._extramsgs.setdefault(name, []).append(msg)


    def on_anymsg(self, msg):
        """Handler for incoming raw message, registers message object."""
        logger.info("Received raw message in %r: %s.", self._anymsg_sub.name, msg)
        self._anymsgs.append(msg)


    def on_service(self, name, req):
        """Handler for service call, registers request object."""
        logger.info("Incoming service call to %r with %s.", name, req)
        self._reqs.setdefault(name, []).append(req)
        return {}


    def test_rospify(self):
        """Tests rospify API."""
        logger.debug("Starting test.")
        self.assertTrue(self._pubs or self._srvs, "No publishers or services configured.")
        resps = {}  # {service name: received resposne objects}

        logger.info("Waiting for subscribers in %s topics.", len(self._pubs))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline \
        and sum(bool(x.get_num_connections()) for x in self._pubs.values()) < len(self._pubs):
            time.sleep(0.5)

        logger.info("Waiting for %s services.", len(self._clis))
        for name, cli in self._clis.items():
            logger.info("Waiting for service %r.", name)
            cli.wait_for_service(5)

        topic = next(iter(self._subs))
        logger.info("Opening subscriber to %r as AnyMsg.", topic)
        self._anymsg_sub = rospy.Subscriber(topic, rospy.AnyMsg, self.on_anymsg)

        for name, pub in self._pubs.items():
            logger.info("Publishing to %r.", name)
            msg = pub.data_class()
            pub.publish(msg)

        for name, cli in self._clis.items():
            logger.info("Invoking service at %r.", name)
            req = cli.request_class()
            resp = cli(req)
            resps.setdefault(name, []).append(resp)


        with self.subTest("rospy"):       self.verify_rospy_index()
        with self.subTest("duration"):    self.verify_duration()
        with self.subTest("time"):        self.verify_time()
        with self.subTest("logging"):     self.verify_logging()
        with self.subTest("masterproxy"): self.verify_masterproxy()
        with self.subTest("publisher"):   self.verify_publisher()
        with self.subTest("subscriber"):  self.verify_subscriber()
        with self.subTest("service"):     self.verify_service()
        with self.subTest("client"):      self.verify_client()
        with self.subTest("timer_rate"):  self.verify_timer_rate()


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
                logger.info("Verifying received messages via added callback in %r.", name)
                self.assertEqual(self._msgs.get(name), self._extramsgs.get(name),
                                 "Expected messages via added callback not received in topic %r."
                                 % name)
                logger.info("Verifying not receiving messages "
                            "via added and removed callback in %r.", name)
                self.assertFalse(self._extramsgs.get("NOT" + name))

        with self.subTest("anymsg"):
            logger.info("Verifying received raw messages in %r.", self._anymsg_sub.name)
            self.assertTrue(self._anymsgs, "No raw messages received.")
            self.assertIsInstance(self._anymsgs[0], rospy.AnyMsg,
                                  "Unexpected type in received raw messages.")


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


    def verify_duration(self):
        """Verifies rospify Duration API."""
        logger.info("Testing Duration API.")
        dur0  = rospy.Duration()
        dur1  = rospy.Duration(secs= 10, nsecs= 20000000)
        dur2  = rospy.Duration(secs= 20, nsecs= 40000000)
        dur1n = rospy.Duration(secs=-10, nsecs=-20000000)
        self.assertEqual(dur1, abs(dur1n),  "Unexpected value for abs(Duration).")
        self.assertEqual(dur1 + dur1, dur2, "Unexpected value for Duration + Duration.")
        self.assertEqual(dur2 - dur1, dur1, "Unexpected value for Duration - Duration.")
        self.assertEqual(dur1 * 2, dur2,    "Unexpected value for Duration * multiplier.")
        self.assertEqual(dur2 / 2, dur1,    "Unexpected value for Duration / divisor.")
        self.assertEqual(dur2 // 2, rospy.Duration(dur1.secs),
                         "Unexpected value for Duration // divisor.")
        self.assertEqual(rospy.Duration(4) % rospy.Duration(3), rospy.Duration(1),
                         "Unexpected value for Duration % Duration.")
        self.assertEqual(divmod(dur2, dur1), (2, dur0),
                         "Unexpected value for divmod(Duration, Duration).")
        with self.assertRaises(ZeroDivisionError, msg="Unexpected success for Duration % zero."):
            dur1 % dur0
        self.assertTrue(bool(dur1),      "Unexpected value for bool(Duration).")
        self.assertFalse(bool(dur0),     "Unexpected value for bool(Duration).")
        self.assertFalse(dur1.is_zero(), "Unexpected value for Duration.is_zero().")
        self.assertTrue(dur0.is_zero(),  "Unexpected value for Duration.is_zero().")
        self.assertEqual(dur1.to_sec(),  dur1.secs + dur1.nsecs / 10**9,
                         "Unexpected value for Duration.to_sec().")
        self.assertEqual(dur1.to_nsec(), dur1.secs * 10**9 + dur1.nsecs,
                         "Unexpected value for Duration.to_sec().")
        durx = copy.copy(dur1)
        durx += dur1
        self.assertEqual(durx, dur2, "Unexpected value for Duration += Duration.")
        durx -= dur1
        self.assertEqual(durx, dur1, "Unexpected value for Duration -= Duration.")
        durx *= 2
        self.assertEqual(durx, dur2, "Unexpected value for Duration *= multiplier.")


    def verify_time(self):
        """Verifies rospify Time API."""
        logger.info("Testing Time API.")
        tm0 = rospy.Time()
        tm1 = rospy.Time(secs=10, nsecs=20000000)
        self.assertTrue(bool(tm1),      "Unexpected value for bool(Time).")
        self.assertFalse(bool(tm0),     "Unexpected value for bool(Time).")
        self.assertFalse(tm1.is_zero(), "Unexpected value for Time.is_zero().")
        self.assertTrue(tm0.is_zero(),  "Unexpected value for Time.is_zero().")
        self.assertEqual(tm1.to_sec(),  tm1.secs + tm1.nsecs / 10**9,
                         "Unexpected value for Time.to_sec().")
        self.assertEqual(tm1.to_nsec(), tm1.secs * 10**9 + tm1.nsecs,
                         "Unexpected value for Time.to_sec().")


    def verify_logging(self):
        """Verifies rospify logging API."""
        funcs = [getattr(rospy, x) for x in dir(rospy)
                 if x.startswith("log") and callable(getattr(rospy, x))]
        logger.info("Testing %s logging functions.", len(funcs))
        msg, msg_throttle = "Invoked %s(msg, %r, %r).", "Invoked %s(period, msg, %r, %r)."
        for func in funcs:
            args = (msg, func, "arg1", "arg2")
            if "_throttle" in func.__name__:  # First argument is throttling period
                args = (10, msg_throttle, func, "arg1", "arg2")
            func(*args)


    def verify_publisher(self):
        """Tests rospify Publisher API."""
        logger.info("Testing Publisher API.")
        for name, pub in self._pubs.items():
            self.assertEqual(pub.name,            name, "Unexpected value for Publisher.name.")
            self.assertEqual(pub.resolved_name,   name, "Unexpected value for Publisher.name.")
            self.assertIsInstance(pub.type,       str,  "Unexpected value for Publisher.type.")
            self.assertIsInstance(pub.data_class, type, "Unexpected value for Publisher.data_class.")
            self.assertIsInstance(pub.md5sum,     str,  "Unexpected value for Publisher.md5sum.")
            self.assertTrue(callable(getattr(pub, "unregister")),
                            "Unexpected value for Publisher.unregister.")
            self.assertTrue(callable(getattr(pub, "get_num_connections")),
                            "Unexpected value for Publisher.get_num_connections.")
            self.assertIsInstance(pub.get_num_connections(), int,
                            "Unexpected value for Publisher.get_num_connections().")


    def verify_subscriber(self):
        """Tests rospify Subscriber API."""
        logger.info("Testing Subscriber API.")
        for name, sub in self._subs.items():
            self.assertEqual(sub.name,            name, "Unexpected value for Subscriber.name.")
            self.assertEqual(sub.resolved_name,   name, "Unexpected value for Subscriber.name.")
            self.assertIsInstance(sub.type,       str,  "Unexpected value for Subscriber.type.")
            self.assertIsInstance(sub.data_class, type, "Unexpected value for Subscriber.data_class.")
            self.assertIsInstance(sub.md5sum,     str,  "Unexpected value for Subscriber.md5sum.")
            self.assertTrue(hasattr(sub, "callback_args"),
                            "Unexpected value for Subscriber.callback_args.")
            self.assertTrue(callable(getattr(sub, "get_num_connections")),
                            "Unexpected value for Subscriber.get_num_connections.")
            self.assertIsInstance(sub.get_num_connections(), int,
                                  "Unexpected value for Subscriber.get_num_connections().")
            self.assertTrue(callable(getattr(sub, "unregister")),
                            "Unexpected value for Subscriber.unregister.")
            self.assertTrue(callable(getattr(sub, "add_callback")),
                            "Unexpected value for Subscriber.add_callback.")
            self.assertTrue(callable(getattr(sub, "remove_callback")),
                            "Unexpected value for Subscriber.remove_callback.")


    def verify_service(self):
        """Tests rospify Service API."""
        logger.info("Testing Service API.")
        for name, srv in self._srvs.items():
            self.assertEqual(srv.resolved_name, name, "Unexpected value for Service.name.")
            self.assertIsInstance(srv.service_class,  type,
                                  "Unexpected value for Service.service_class.")
            self.assertIsInstance(srv.request_class,  type,
                                  "Unexpected value for Service.request_class.")
            self.assertIsInstance(srv.response_class, type,
                                  "Unexpected value for Service.response_class.")
            self.assertTrue(callable(getattr(srv, "spin")),
                            "Unexpected value for Service.spin.")
            self.assertTrue(callable(getattr(srv, "shutdown")),
                            "Unexpected value for Subscriber.shutdown.")


    def verify_client(self):
        """Tests rospify ServiceProxy API."""
        logger.info("Testing ServiceProxy API.")
        for name, cli in self._clis.items():
            self.assertEqual(cli.resolved_name, name, "Unexpected value for ServiceProxy.name.")
            self.assertIsInstance(cli.service_class,  type,
                                  "Unexpected value for ServiceProxy.service_class.")
            self.assertIsInstance(cli.request_class,  type,
                                  "Unexpected value for ServiceProxy.request_class.")
            self.assertIsInstance(cli.response_class, type,
                                  "Unexpected value for ServiceProxy.response_class.")
            self.assertTrue(callable(cli), "Unexpected value for ServiceProxy.__call__.")
            self.assertTrue(callable(getattr(cli, "close")),
                            "Unexpected value for ServiceProxy.close.")


    def verify_timer_rate(self):
        """Verifies rospify Timer and Rate API."""
        logger.info("Testing Timer and Rate API.")
        lists = {"t1": [], "t2": []}
        t1 = rospy.Timer(rospy.Duration(1),   lists["t1"].append)
        t2 = rospy.Timer(rospy.Duration(0.5), lists["t2"].append, oneshot=True)
        r  = rospy.Rate(0.5)
        r.sleep()
        self.assertTrue(callable(getattr(r, "remaining")),
                        "Unexpected value for Rate.remaining.")
        self.assertIsInstance(r.remaining(), rospy.Duration,
                              "Unexpected value for Rate.remaining().")
        self.assertTrue(callable(getattr(t1, "is_alive")),
                        "Unexpected value for Timer.is_alive.")
        self.assertGreater(len(lists["t1"]), 1, "Timer fired insufficient times.")
        self.assertEqual(len(lists["t2"]), 1, "One-shot timer fired more than once.")
        self.assertIsInstance(lists["t1"][-1], rospy.TimerEvent,
                              "Unexpected value type given to timer callback.")
        self.assertTrue(callable(getattr(t1, "shutdown")),
                        "Unexpected value for Timer.shutdown.")
        t1.shutdown()
        self.assertFalse(t1.is_alive(), "Unexpected value for Timer.is_alive().")
        self.assertFalse(t2.is_alive(), "Unexpected value for Timer.is_alive().")
        t1.join()
        t2.join()


    def verify_masterproxy(self):
        """Verifies rospify MasterProxy API."""
        logger.info("Testing MasterProxy API.")
        master = rospy.get_master()
        ret, msg, state = master.getSystemState()
        self.assertIsInstance(ret, int,
                              "Unexpected return code from MasterProxy.getSystemState().")
        self.assertIsInstance(msg, str,
                              "Unexpected return message from MasterProxy.getSystemState().")
        self.assertEqual(len(state), 3,
                         "Unexpected result from MasterProxy.getSystemState().")
        pubs, subs, srvs = state
        self.assertTrue(any(n in self._pubs for n, _ in pubs),
                        "Published topics not present in MasterProxy.getSystemState().")
        self.assertTrue(any(rospy.get_name() in nn for _, nn in pubs),
                        "Node name not present in MasterProxy.getSystemState().")
        self.assertTrue(any(n in self._subs for n, _ in subs),
                        "Subscribed topics not present in MasterProxy.getSystemState().")
        self.assertTrue(any(rospy.get_name() in nn for _, nn in subs),
                        "Node name not present in MasterProxy.getSystemState().")
        self.assertTrue(any(n in self._srvs for n, _ in srvs),
                        "Opened services not present in MasterProxy.getSystemState().")
        self.assertTrue(any(rospy.get_name() in nn for _, nn in srvs),
                        "Node name not present in MasterProxy.getSystemState().")

        ret, msg, topicstypes = master.getPublishedTopics("/test/topic")
        self.assertIsInstance(ret, int,
                              "Unexpected return code from MasterProxy.getPublishedTopics().")
        self.assertIsInstance(msg, str,
                              "Unexpected return message from MasterProxy.getPublishedTopics().")
        self.assertIsInstance(topicstypes, list,
                              "Unexpected result from MasterProxy.getPublishedTopics().")
        self.assertTrue(any(n in self._pubs for n, _ in topicstypes),
                        "Published topics not present in MasterProxy.getPublishedTopics().")
        self.assertFalse(any(not n.startswith("/test/topic") for n, _ in topicstypes),
                        "Subgraph parameter not respected in MasterProxy.getPublishedTopics(): %s."
                        % topicstypes)
        self.assertTrue(all(t == rosros.api.canonical(t) for _, t in topicstypes),
                        "Topic types not canonical in MasterProxy.getPublishedTopics().")

        ret, msg, topicstypes = master.getTopicTypes()
        self.assertIsInstance(ret, int,
                              "Unexpected return code from MasterProxy.getTopicTypes().")
        self.assertIsInstance(msg, str,
                              "Unexpected return message from MasterProxy.getTopicTypes().")
        self.assertIsInstance(topicstypes, list,
                              "Unexpected result from MasterProxy.getTopicTypes().")
        self.assertTrue(any(n in self._pubs for n, _ in topicstypes),
                        "Published topics not present in MasterProxy.getTopicTypes().")
        self.assertTrue(any(n in self._subs for n, _ in topicstypes),
                        "Subscribed topics not present in MasterProxy.getTopicTypes().")
        self.assertTrue(all(t == rosros.api.canonical(t) for _, t in topicstypes),
                        "Topic types not canonical in MasterProxy.getTopicTypes().")


    def verify_rospy_index(self):
        """Tests rospify main functions."""
        logger.info("Testing rospify index API.")

        FUNCS = [
            "delete_param", "get_caller_id", "get_master", "get_name", "get_namespace",
            "get_node_uri", "get_param", "get_param_names", "get_param_cached",
            "get_published_topics", "get_ros_root", "get_rostime", "get_time",
            "has_param", "init_node", "is_shutdown", "myargv", "on_shutdown",
            "parse_rosrpc_uri", "remap_name", "resolve_name", "search_param",
            "set_param", "signal_shutdown", "sleep", "spin", "wait_for_message",
            "wait_for_service"
        ]
        for name in FUNCS:
            self.assertTrue(callable(getattr(rospy, name, None)),
                            "No function %r in rospy API." % name)


        self.assertEqual(rospy.get_name(), "/" + self.NAME, "Unexpected value from get_name().")
        self.assertEqual(rospy.get_namespace(), "/",     "Unexpected value from get_namespace().")
        self.assertIsInstance(rospy.get_node_uri(), str, "Unexpected type from get_node_uri().")
        self.assertIsInstance(rospy.get_ros_root(), str, "Unexpected type from get_ros_root().")

        self.assertEqual(rospy.parse_rosrpc_uri("rosrpc://host:1234"), ("host", 1234),
                         "Unexpected value from parse_rosrpc_uri().")

        topics = rospy.get_published_topics()
        self.assertIsInstance(topics, list,  "Unexpected type from get_published_topics().")
        self.assertTrue(topics,              "No topics from get_published_topics().")
        topicmap = dict(topics)
        for name, pub in self._pubs.items():
            self.assertIn(name, topicmap, "Published topic not present in get_published_topics().")
            self.assertEqual(topicmap[name], pub.type,
                             "Unexpected typename from get_published_topics().")


        self.assertIsInstance(rospy.get_rostime(), rospy.Time, "Unexpected type from get_rostime().")
        self.assertAlmostEqual(rospy.get_time(), time.time(), delta=1,
                               msg="Unexpected value from get_time().")
        self.assertFalse(rospy.is_shutdown(), "Unexpected value from is_shutdown().")


        name, value = self.NAME, 123
        self.assertEqual(rospy.get_param("~" + name, value), value,
                         "Unexpected result from get_param().")
        rospy.set_param("~" + name, value)
        self.assertTrue(rospy.has_param("~" + name),         "Unexpected result from has_param().")
        self.assertEqual(rospy.get_param("~" + name), value, "Unexpected result from get_param().")
        self.assertEqual(rospy.get_param_cached("~" + name), value,
                         "Unexpected result from get_param_cached().")
        self.assertEqual(rospy.search_param(name), "/%s/%s" % (self.NAME, name),
                         "Unexpected result from search_param().")
        self.assertIn("/%s/%s" % (self.NAME, name), rospy.get_param_names(),
                      "Unexpected result from get_param_names().")
        rospy.delete_param("~" + name)
        with self.assertRaises(KeyError, msg="Unexpected success from get_param()."):
            rospy.get_param("~" + name)

        self.assertIsInstance(rospy.myargv(), list, "Unexpected type from myargv().")

        try:
            rospy.on_shutdown(lambda: None)
        except Exception as e:
            self.fail("Unexpected error from on_shutdown(): %s\n%s" % (e, traceback.format_exc()))

        self.assertEqual(rospy.resolve_name("x"), "/x", "Unexpected result from resolve_name().")
        self.assertEqual(rospy.resolve_name("~x"), "/%s/x" % self.NAME,
                         "Unexpected result from resolve_name().")
        self.assertEqual(rospy.resolve_name("~x", "/top"), "/top/x",
                         "Unexpected result from resolve_name().")

        for topic1, topic2 in self.REMAPS.items():
            self.assertEqual(rospy.remap_name(topic1), topic2, "Unexpected result from remap_name().")

        start = time.time()
        rospy.sleep(1)
        self.assertAlmostEqual(time.time() - 1, start, delta=0.2,
                               msg="Unexpected elapsed time from sleep().")

        for name in self._clis:
            try:
                rospy.wait_for_service(name, timeout=1)
            except Exception as e:
                self.fail("Unexpected error from wait_for_service(): %s\n%s" %
                          (e, traceback.format_exc()))

        logger.info("Testing wait_for_message().")
        for name1, name2 in zip(self._pubs, self._subs):
            logger.info("Publishing to %r.", name1)
            pub, sub = self._pubs[name1], self._subs[name2]
            def delay_publish(m): time.sleep(1), pub.publish(m)
            threading.Thread(target=delay_publish, args=(pub.data_class(), )).start()
            logger.info("Waiting for message in %r.", name2)
            rospy.wait_for_message(name2, sub.data_class, 5)


if "__main__" == __name__:
    TestRospify.run_rostest()
