#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test: core functions.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     09.12.2023
@modified    09.12.2023
------------------------------------------------------------------------------
"""
import logging
import os
import sys
import time

from rosros import api, core

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from test import testbase

logger = logging.getLogger()


class TestCore(testbase.TestBase):
    """Tests core functions."""

    ## Test name used in flow logging
    NAME = "test_core"

    ## Invoked callback arguments [(args, kwargs), ]
    CALLS = []


    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.init_local_node()


    def test_core(self):
        """Tests core functions"""
        NAME = lambda f, *a: "%s.%s(%s)" % (f.__module__, (f.__name__), ", ".join(map(repr, a)))
        ERR  = lambda f, *a: "Unexpected result from %s(%s)." % (f.__name__, ", ".join(map(repr, a)))

        CALLS = []
        CALLBACKS = [TestCore.cb_static, self.cb_static, TestCore.cb_class, self.cb_class,
                     self.cb_instance, lambda *a, **w: self.CALLS.append((a, w))]
        func = core.on_shutdown
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            for i, cb in enumerate(CALLBACKS):
                ARGS = [((), {}), ((i, cb), {}), ((), {"i": i, "cb": cb}),
                        ((i, ), {"cb": cb})]
                for args, kwargs in ARGS:
                    logger.debug("Testing %s with %s(*%r, **%r).", NAME(func), cb, args, kwargs)
                    CALLS.append((args, kwargs))
                    func(cb, *args, **kwargs)

        core.start_spin()

        DURATIONS = sum(([x, api.make_duration(x)] for x in (1, 0, -1)), [])
        func = core.sleep
        with self.subTest(NAME(func)):
            logger.info("Testing %s.", NAME(func))
            for duration in DURATIONS:
                logger.info("Testing %s.", NAME(func, duration))
                stamp1 = time.monotonic()
                func(duration)
                stamp2 = time.monotonic()
                self.assertAlmostEqual(stamp2 - stamp1, max(0, api.to_sec(duration)), delta=1/3,
                                       msg="Unexpected return time from %s." % ERR(func, duration))

        core.shutdown()
        time.sleep(0.5)

        func = core.on_shutdown
        with self.subTest(NAME(func)):
            logger.info("Verifying %s results.", NAME(func))
            for i, cb in enumerate(CALLBACKS):
                ARGS = [((), {}), ((i, cb), {}), ((), {"i": i, "cb": cb}),
                        ((i, ), {"cb": cb})]
                for j, (args, kwargs) in enumerate(ARGS):
                    logger.debug("Verifying %s for %s(*%r, **%r).", NAME(func), cb, args, kwargs)
                    idx = i * len(ARGS) + j
                    received = self.CALLS[idx] if idx < len(self.CALLS) else None
                    self.assertEqual(received, CALLS[idx],
                                     "Callback to %s for %s(*%r, **%r) not invoked." %
                                     (NAME(func), cb, args, kwargs))


    @staticmethod
    def cb_static(*args, **kwargs): TestCore.CALLS.append((args, kwargs))

    @classmethod
    def cb_class(cls, *args, **kwargs): cls.CALLS.append((args, kwargs))

    def cb_instance(self, *args, **kwargs): self.CALLS.append((args, kwargs))



    __all__ = [
    "create_client", "create_publisher", "create_rate", "create_service", "create_subscriber",
    "create_timer", "delete_param", "destroy_entity", "get_logger", "get_namespace",
    "get_node_name", "get_nodes", "get_param", "get_param_names", "get_params", "get_rostime",
    "get_services", "get_topics", "has_param", "init_node", "init_params", "ok", "on_shutdown",
    "register_init", "remap_name", "resolve_name", "set_param", "sleep", "shutdown",
    "spin", "spin_once", "spin_until_future_complete", "start_spin",
    "wait_for_publisher", "wait_for_subscriber", "wait_for_service"
]




if "__main__" == __name__:
    TestCore.run_rostest()
