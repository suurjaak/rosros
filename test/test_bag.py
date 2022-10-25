#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test: ROS bag functionality.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     23.10.2022
@modified    25.10.2022
------------------------------------------------------------------------------
"""
import logging
import os
import sys
import time
import tempfile

import std_msgs.msg

import rosros
from rosros import api

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from test import testbase


logger = logging.getLogger()


## Returns an instance ROS time type for messages
make_stamp = lambda x: api.time_message(api.make_time(x))



class TestBag(testbase.TestBase):
    """Tests Bag API."""

    ## Test name used in flow logging
    NAME = "test_bag"


    ## Bag data as {topic: {type: {name: value or callable(seq)}}}
    DATA = {
        "/topic/bool0":  {"std_msgs/Bool":   {"data":     False}},
        "/topic/bool1":  {"std_msgs/Bool":   {"data":     True}},
        "/topic/header": {"std_msgs/Header": {"frame_id": NAME, "stamp": make_stamp}},
    }
    ## Bag message counts as {topic: {type: count}}
    COUNTS = {
        "/topic/bool0":  {"std_msgs/Bool":    5},
        "/topic/bool1":  {"std_msgs/Bool":    7},
        "/topic/header": {"std_msgs/Header": 11},
    }
    ## Message classes {typename: typeclass}
    CLASSES = {
        "std_msgs/Bool":   std_msgs.msg.Bool,
        "std_msgs/Header": std_msgs.msg.Header,
    }


    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._name1     = None  # Path to 1st bag
        self._name2     = None  # Path to 2nd bag
        self._expecteds = {}    # {filename: {(topic, typename): [{msg values}, ]}}
        self._stamps    = {}    # {filename: {(topic, typename): [float or None if default, ]}}

    def setUp(self):
        """Creates temporary files for bags."""
        super().setUp()
        with tempfile.NamedTemporaryFile(suffix=rosros.Bag.EXTENSION) as f: self._name1 = f.name
        with tempfile.NamedTemporaryFile(suffix=rosros.Bag.EXTENSION) as f: self._name2 = f.name


    def tearDown(self):
        """Deletes temoorary files."""
        try: os.remove(self._name1)
        except Exception: pass
        try: os.remove(self._name2)
        except Exception: pass
        super().tearDown()


    def test_bag(self):
        """Tests bag interfaces."""

        logger.info("Verifying writing to two separate bags.")
        with rosros.Bag(self._name1, "w") as bag1, rosros.Bag(self._name2, mode="w") as bag2:
            for b, d in ((b, d) for b in (bag1, bag2) for d in (self._expecteds, self._stamps)):
                d.setdefault(b.filename, {})
            seq, topicseq, MAXTOPICS = 0, 0, sum(len(dd) for dd in self.DATA.values())
            for topic, typename, data in ((t, n, d) for t, dd in self.DATA.items()
                                          for n, d in dd.items()):
                key, count, raw = (topic, typename), self.COUNTS[topic][typename], not self._expecteds
                logger.info("Writing %s messages in %r as %s%r.",
                            count, topic, "raw " if raw else "", typename)
                cls = rosros.api.get_message_class(typename)
                for _ in range(count):
                    vals = {k: v(seq) if callable(v) else v for k, v in data.items()}
                    msg = cls(**vals)
                    if raw:
                        typehash = api.get_message_type_hash(cls)
                        msg = (typename, api.serialize_message(msg), typehash, cls)
                    stamp = make_stamp(time.time() - 100) if not topicseq else \
                            make_stamp(time.time() + 100) if topicseq == MAXTOPICS - 1 else None
                    bag = bag1 if seq % 2 else bag2
                    bag.write(topic, msg, stamp, raw=raw, qoses=None)
                    self._expecteds[bag.filename].setdefault(key, []).append(vals)
                    self._stamps   [bag.filename].setdefault(key, []).append(api.to_sec(stamp))
                    seq += 1
                topicseq += 1

        bag1, bag2 = rosros.Bag(self._name1, "r"), rosros.Bag(self._name2, "r")
        logger.info("Bag #1 metainfo:\n%s", bag1)
        logger.info("Bag #2 metainfo:\n%s", bag2)

        logger.info("Verifying Bag.write() failing in read-only mode.")
        for bag in (bag1, bag2):
            with self.subTest("nowrite"):
                self.verify_nowrite(bag)

        logger.info("Verifying Bag global getters .get_start_time() et al.")
        for bag in (bag1, bag2):
            with self.subTest("globals"):
                self.verify_globals(bag)

        logger.info("Verifying Bag message type getters.")
        for bag in (bag1, bag2):
            with self.subTest("message_types"):
                self.verify_message_types(bag)

        logger.info("Verifying Bag.get_topic_info().")
        for bag in (bag1, bag2):
            with self.subTest("get_topic_info"):
                self.verify_get_topic_info(bag)

        logger.info("Verifying Bag.get_type_and_topic_info().")
        for bag in (bag1, bag2):
            with self.subTest("get_type_and_topic_info"):
                self.verify_get_type_and_topic_info(bag)

        logger.info("Verifying bag message contents.")
        for bag in (bag1, bag2):
            with self.subTest("messages"):
                self.verify_messages(bag)

        logger.info("Verifying bag message timestamps.")
        for bag in (bag1, bag2):
            with self.subTest("stamps"):
                self.verify_stamps(bag)

        logger.info("Verifying reading raw messages from bag.")
        for bag in (bag1, bag2):
            with self.subTest("messages_raw"):
                self.verify_messages_raw(bag)

        bag1.close(), bag2.close()


    def verify_nowrite(self, bag):
        """Verifies failure when trying to write to a read-only bag."""
        with self.assertRaises(Exception, msg="Unexpected success on writing to read-only bag."):
            bag.write("/some/topic", std_msgs.msg.Bool())


    def verify_globals(self, bag):
        """Verifies global data getters like total count."""
        self.assertEqual(bag.get_message_count(),
                         sum(len(mm) for mm in self._expecteds[bag.filename].values()),
                         "Unexpected result from Bag.get_message_count().")
        self.assertEqual(bag.get_start_time(),
                         min(t for tt in self._stamps[bag.filename].values() for t in tt if t is not None),
                         "Unexpected result from Bag.get_start_time().")
        self.assertEqual(bag.get_end_time(),
                         max(t for tt in self._stamps[bag.filename].values() for t in tt if t is not None),
                         "Unexpected result from Bag.get_end_time().")

        self.assertFalse(bag.get_qoses(*next((t, n) for t, dd in self.DATA.items() for n in dd)),
                        "Unexpected result from Bag.get_qoses().")


    def verify_message_types(self, bag):
        """Verifies Bag.get_message_xyz() methods."""
        for typename, cls in self.CLASSES.items():
            cls1 = bag.get_message_class(typename)
            self.assertEqual(api.get_message_type(cls1), api.get_message_type(cls),
                             "Unexpected result from Bag.get_message_class(%r)." % typename)
            self.assertEqual(bag.get_message_definition(typename), api.get_message_definition(cls),
                             "Unexpected result from Bag.get_message_definition(%r)." % typename)
            self.assertEqual(bag.get_message_type_hash(typename), api.get_message_type_hash(cls),
                             "Unexpected result from Bag.get_message_type_hash(%r)." % typename)


    def verify_get_topic_info(self, bag):
        """Verifies Bag.get_topic_info() method."""
        info = bag.get_topic_info()
        for topic, typename in ((t, n) for t, dd in self.DATA.items() for n in dd):
            typehash = api.get_message_type_hash(typename)
            key = (topic, typename, typehash)
            self.assertIn(key, info, "Unexpected result from Bag.get_topic_info().")
            self.assertEqual(info[key], len(self._expecteds[bag.filename][(topic, typename)]),
                             "Unexpected result from Bag.get_topic_info().")


    def verify_get_type_and_topic_info(self, bag):
        """Verifies Bag.get_type_and_topic_info() method."""
        info = bag.get_type_and_topic_info()
        self.assertIsInstance(info, tuple, "Unexpected result from Bag.get_type_and_topic_info().")
        self.assertIsInstance(info.msg_types, dict,
                              "Unexpected result from Bag.get_type_and_topic_info().")
        self.assertIsInstance(info.topics, dict,
                              "Unexpected result from Bag.get_type_and_topic_info().")


    def verify_messages(self, bag):
        """Verifies bag message contents."""
        outputs = {}  # {filename: {(topic, typename): [msg, ]}}
        for topic, msg, _ in bag:
            outputs.setdefault((topic, api.get_message_type(msg)), []).append(msg)
        for (topic, typename), expected in self._expecteds[bag.filename].items():
            msgs = outputs.get((topic, typename))
            datas = [{k: getattr(m, k) for k in e} for m, e in zip(msgs, expected)]
            self.assertEqual(datas, expected, "Unexpected content read in topic %s." % topic)


    def verify_stamps(self, bag):
        """Verifies bag message timestamps."""
        for (topic, typename), stamps in self._stamps[bag.filename].items():
            if not any(stamps): continue  # for
            outputs, expected = [], [None if x is None else api.make_time(x) for x in stamps]
            for topic2, msg, stamp2 in bag.read_messages([topic]):
                self.assertEqual(topic2, topic,
                                "Unexpected topic from Bag.read_messages(%r)." % topic)
                if api.get_message_type(msg) == typename:
                    outputs.append(stamp2)
            outputs, expected = zip(*((a, b) for a, b in zip(expected, outputs) if a is not None))
            self.assertEqual(outputs, expected,
                             "Unexpected timestamps from bag topic %r (%s)" % (topic, typename))


    def verify_messages_raw(self, bag):
        """Verifies reading bag messages as raw data."""
        for (topic, typename), expected in self._expecteds[bag.filename].items():
            msgs = []
            cls, typehash = self.CLASSES[typename], api.get_message_type_hash(typename)
            for bagtopic, msg, _ in bag.read_messages(topic, raw=True):
                self.assertEqual(bagtopic, topic,
                    "Unexpected topic from Bag.read_messages(%r)." % topic)
                self.assertIsInstance(msg, tuple,  # (typename, bytes, typehash, ?.., typeclass)
                    "Unexpected result from Bag.read_messages(%r, raw=True)." % topic)
                self.assertGreaterEqual(len(msg), 4,
                    "Unexpected result from Bag.read_messages(%r, raw=True)." % topic)
                self.assertEqual(msg[0], typename,
                    "Unexpected typename from Bag.read_messages(%r)." % topic)
                self.assertIsInstance(msg[1], bytes,
                    "Unexpected binary data type from Bag.read_messages(%r)." % topic)
                self.assertEqual(msg[2], typehash,
                    "Unexpected typehash from Bag.read_messages(%r)." % topic)
                self.assertEqual(api.get_message_type(msg[-1]), api.get_message_type(cls),
                    "Unexpected typeclass from Bag.read_messages(%r)." % topic)
                msgs.append(api.deserialize_message(msg[1], cls))

            datas = [{k: getattr(m, k) for k in e} for m, e in zip(msgs, expected)]
            self.assertEqual(datas, expected, "Unexpected raw content read in topic %s." % topic)


if "__main__" == __name__:
    TestBag.run_rostest()
