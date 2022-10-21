#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test: publish and subscribe to topics with rosros unified API.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     12.02.2022
@modified    21.10.2022
------------------------------------------------------------------------------
"""
import functools
import logging
import os
import sys
import time

import rosros

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from test import testbase, testnode

logger = logging.getLogger()


class TestTopics(testbase.TestBase):
    """Tests publishing and subscribing."""

    ## Test name used in flow logging
    NAME = "test_topics"


    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._pubs      = {}    # {topic name: publisher instance}
        self._subs      = {}    # {topic name: subscriber instance}
        self._msgs      = {}    # {topic name: [incoming message, ]}
        self._exps      = {}    # {topic name: expected message data dictionary}
        self._raw_sub   = None  # subscriber instance for raw messages
        self._latch_pub = None  # publisher instance for latched messages
        self._raw_cls   = {}    # {topic name: real message class}

        self.init_local_node()


    def setUp(self):
        """Opens publishers and subscribers."""
        super().setUp()
        for opts in testnode.DEFAULTS.get("subscribe", {}).values():
            if "action" not in opts or "publish" != opts["action"].get("category"):
                continue  # for opts
            raw = not self._raw_sub
            latch = not self._latch_pub
            action = opts["action"]
            handler = functools.partial(self.on_message, action["name"])
            logger.info("Opening subscriber to %r as %s.",
                        action["name"], "raw " if raw else action["type"])
            sub = rosros.create_subscriber(action["name"], action["type"], handler, raw=raw)
            logger.info("Opening %spublisher to %r as %s.",
                        "latched " if latch else "", opts["name"], opts["type"])
            pub = rosros.create_publisher(opts["name"], opts["type"], latch=latch)
            self._subs[action["name"]], self._pubs[opts["name"]] = sub, pub
            self._exps[action["name"]] = action.get("value", {})
            if raw:
                self._raw_sub = sub
                self._raw_cls[action["name"]] = rosros.api.get_message_class(action["type"])
            if latch:
                self._latch_pub = pub

        logger.info("Publishing to latched %r.", self._latch_pub.name)
        self._latch_pub.publish(self._latch_pub.data_class())

        self.run_test_node()
        rosros.start_spin()


    def tearDown(self):
        """Shuts down publishers and subscribers.."""
        for k in list(self._pubs): self._pubs.pop(k).unregister()
        for k in list(self._subs): self._subs.pop(k).unregister()
        super().tearDown()


    def on_message(self, name, msg):
        """Handler for incoming message, registers message object."""
        loggable = msg if "*" != rosros.api.get_message_type(msg) else "AnyMsg"
        # rospy.AnyMsg raises error on string representation
        logger.info("Received message in %r: %s.", name, loggable)
        self._msgs.setdefault(name, []).append(msg)


    def test_topics(self):
        """Tests publishing and subscribing to topics."""
        logger.debug("Starting test.")
        self.assertTrue(self._pubs, "No publishers configured.")

        logger.info("Waiting for subscribers in %s topics.", len(self._pubs))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline \
        and sum(bool(x.get_num_connections()) for x in self._pubs.values()) < len(self._pubs):
            rosros.spin_once(1)

        for name, pub in self._pubs.items():
            if pub is self._latch_pub: continue  # for name, pub
            logger.info("Publishing to %r.", name)
            msg = pub.data_class()
            pub.publish(msg)

        logger.info("Waiting for messages in %s topics.", len(self._subs))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline \
        and sum(bool(x) for x in self._msgs.values()) < len(self._subs):
            rosros.spin_once(1)

        logger.info("Verifying received messages in %s topics.", len(self._subs))
        for name, sub in self._subs.items():
            logger.info("Verifying received %smessages in %r.",
                        "raw " if sub is self._raw_sub else "", name)
            with self.subTest(name):
                self.assertTrue(self._msgs.get(name),
                                "Expected message not received in topic %r." % name)
                msg = self._msgs[name][0]
                if sub is self._raw_sub:
                    msg = rosros.api.deserialize_message(msg, self._raw_cls[name])
                for k, v in self._exps[name].items():
                    if isinstance(v, (dict, list)): continue  # for k, v
                    logger.info("Verifying %s=%r in %r.", k, v, name)
                    self.assertEqual(rosros.util.get_value(msg, k), v,
                                     "Unexpected value in %s." % k)


if "__main__" == __name__:
    TestTopics.run_rostest()
