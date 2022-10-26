#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test: ROS bag functionality.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     23.10.2022
@modified    26.10.2022
------------------------------------------------------------------------------
"""
import collections
import logging
import os
import random
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
        "/topic/bool0":  {"std_msgs/Bool":   {"data":     lambda x: bool(x % 2)}},
        "/topic/bool1":  {"std_msgs/Bool":   {"data":     lambda x: not x % 2}},
        "/topic/header": {"std_msgs/Header": {"frame_id": lambda x: "%s_%s" % (TestBag.NAME, x),
                                              "stamp":    make_stamp}},
    }
    ## Bag message counts as {topic: {type: count}}
    COUNTS = {
        "/topic/bool0":  {"std_msgs/Bool":   11},
        "/topic/bool1":  {"std_msgs/Bool":   13},
        "/topic/header": {"std_msgs/Header": 17},
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
        # {filename: {(topic, typename): [{msg values}, ]}}
        self._expecteds = collections.defaultdict(lambda: collections.defaultdict(list))
        # {filename: {(topic, typename): [(time.time(), float given to write() or None), ]}}
        self._stamps    = collections.defaultdict(lambda: collections.defaultdict(list))

    def setUp(self):
        """Creates temporary files for bags."""
        super().setUp()
        with tempfile.NamedTemporaryFile(suffix=rosros.Bag.EXTENSION) as f: self._name1 = f.name
        with tempfile.NamedTemporaryFile(suffix=rosros.Bag.EXTENSION) as f: self._name2 = f.name


    def tearDown(self):
        """Deletes temoorary files."""
        for filename in (self._name1, self._name2):
            try: os.remove(filename)
            except Exception: pass
        super().tearDown()


    def test_bag(self):
        """Tests bag interfaces."""
        rander = random.Random(0)
        writeables = {(t, n): c for t, d in self.COUNTS.items() for n, c in d.items()}
        def choose_write_topic():
            result = None
            if any(writeables.values()):
                result = rander.choice([k for k, c in writeables.items() if c])
                writeables[result] -= 1
            return result

        logger.info("Verifying writing to two separate bags.")
        with rosros.Bag(self._name1, "w") as bag1, rosros.Bag(self._name2, mode="w") as bag2:
            topickey, seq = choose_write_topic(), 0
            while topickey:
                bag = bag1 if seq % 2 else bag2
                self.write_message(bag, seq, *topickey)
                topickey = choose_write_topic()
                time.sleep(0.01)  # Spread out timestamps
                seq += 1
        logger.info("Wrote %s messages to bag #1, %s messages to bag #2.",
                    sum(len(tt) for tt in self._stamps[bag1.filename].values()),
                    sum(len(tt) for tt in self._stamps[bag2.filename].values()))

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

        logger.info("Verifying Bag.read_messages(start_time, end_time).")
        for bag in (bag1, bag2):
            with self.subTest("messages_timefilter"):
                self.verify_messages_timefilter(bag)

        logger.info("Verifying reading raw messages from bag.")
        for bag in (bag1, bag2):
            with self.subTest("messages_raw"):
                self.verify_messages_raw(bag)

        bag1.close(), bag2.close()


    def write_message(self, bag, seq, topic, typename):
        """Writes a message to bag, updates state dicts."""
        TOTAL = sum(sum(d.values()) for d in self.COUNTS.values())
        stampdelta = (-100 if seq < TOTAL // 4 else 100 if seq > 3 * TOTAL // 4 else None)
        cls, raw = rosros.api.get_message_class(typename), seq % 2
        vals = {k: v(seq) if callable(v) else v for k, v in self.DATA[topic][typename].items()}
        msg = cls(**vals)
        if raw:
            typehash = api.get_message_type_hash(cls)
            msg = (typename, api.serialize_message(msg), typehash, cls)
        realstamp = time.time()
        bag_stamp = realstamp + stampdelta if stampdelta else None
        bag.write(topic, msg, bag_stamp, raw=raw, qoses=None)
        stampentry = (realstamp, realstamp + stampdelta if stampdelta else None)  # (actual, given)
        self._expecteds[bag.filename][(topic, typename)].append(vals)
        self._stamps   [bag.filename][(topic, typename)].append(stampentry)


    def verify_nowrite(self, bag):
        """Verifies failure when trying to write to a read-only bag."""
        with self.assertRaises(Exception, msg="Unexpected success on writing to read-only bag."):
            bag.write("/some/topic", std_msgs.msg.Bool())


    def verify_globals(self, bag):
        """Verifies global data getters like total count."""
        alltopics = [t for t, _ in self._expecteds[bag.filename]]
        self.assertEqual(bag.get_message_count(),
                         sum(len(mm) for mm in self._expecteds[bag.filename].values()),
                         "Unexpected result from Bag.get_message_count().")
        self.assertEqual(bag.get_message_count(alltopics),
                         sum(len(mm) for mm in self._expecteds[bag.filename].values()),
                         "Unexpected result from Bag.get_message_count(%s)." % alltopics)
        for i, ((topic, _), mm) in enumerate(self._expecteds[bag.filename].items()):
            self.assertEqual(bag.get_message_count(topic if i % 2 else [topic]), len(mm),
                             "Unexpected result from Bag.get_message_count(%s)." % topic)

        self.assertEqual(bag.get_start_time(),
                         min(g or r for tt in self._stamps[bag.filename].values() for r, g in tt),
                         "Unexpected result from Bag.get_start_time().")
        self.assertEqual(bag.get_end_time(),
                         max(g or r for tt in self._stamps[bag.filename].values() for r, g in tt),
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
            outputs = []
            expected_almost = [api.make_time(r) for r, g in stamps if g is None]
            expected_exact  = [api.make_time(g) for _, g in stamps if g is not None]
            for topic2, msg, stamp2 in bag.read_messages([topic]):
                self.assertEqual(topic2, topic,
                                "Unexpected topic from Bag.read_messages(%r)." % topic)
                if api.get_message_type(msg) == typename:
                    outputs.append(stamp2)

            outputs_almost = [a for a, (r, g) in zip(outputs, stamps) if g is None]
            outputs_exact  = [a for a, (r, g) in zip(outputs, stamps) if g is not None]

            self.assertEqual(outputs_exact, expected_exact,
                             "Unexpected timestamps from bag topic %r (%s)" % (topic, typename))
            for output, expected in zip(outputs_almost, expected_almost):
                self.assertAlmostEqual(
                    api.to_sec(output), api.to_sec(expected), places=1,
                    msg="Unexpected timestamp from bag topic %r (%s)" % (topic, typename))


    def verify_messages_timefilter(self, bag):
        """Verifies Bag.read_messages(start_time, end_time)."""
        topickeys = [k for k, tt in self._stamps[bag.filename].items() if any(g for r, g in tt)][:2]
        topics = [topic for topic, _ in topickeys]
        tstamps = sorted(g for k in topickeys for _, g in self._stamps[bag.filename][k]
                         if g is not None)
        # Choose start-end so that all selected topics will yield some messages
        t1 = next(s for s in tstamps[1:] if all(
            any(g >= s for _, g in self._stamps[bag.filename][k] if g is not None
        ) for k in topickeys))
        t2 = next(s for s in tstamps[-2::-1] if all(
            any(g <= s for _, g in self._stamps[bag.filename][k] if g is not None
        ) for k in topickeys))

        bag_outputs = {}  # {topic: [stamp, ]}
        for topic, _, stamp in bag.read_messages(topics, start_time=t1, end_time=t2):
            self.assertIn(topic, topics,
                          "Unexpected topic from Bag.read_messages(%r)." % topics)
            bag_outputs.setdefault(topic, []).append(stamp)
        for outputs in bag_outputs.values():
            self.assertTrue(all(t1 <= x <= t2 for x in map(api.to_sec, outputs)),
                "Unexpected results from Bag.read_messages(start_time=%s, end_time=%s): %s." %
                (t1, t2, list(map(api.to_sec, outputs))))
        for key, stamps in (x for x in self._stamps[bag.filename].items() if x[0] in topickeys):
            stamps = [(r, g) for r, g in stamps if t1 <= (r if g is None else g) <= t2]
            outputs = [a for a, (_, g) in zip(bag_outputs[key[0]], stamps) if g is not None]
            expected = [api.make_time(g) for _, g in stamps if g is not None and t1 <= g <= t2]
            self.assertEqual(outputs, expected,
                            "Unexpected results from Bag.read_messages(start_time, end_time).")


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
