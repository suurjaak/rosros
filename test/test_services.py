#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test: invoke and provide services with rosros unified API.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     12.04.2022
@modified    09.12.2022
------------------------------------------------------------------------------
"""
import functools
import logging
import os
import sys
import time

import rosros

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from test import testbase, testnode

logger = logging.getLogger()


class TestServices(testbase.TestBase):
    """Tests publishing and subscribing."""

    ## Test name used in flow logging
    NAME = "test_services"


    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._clis = {}  # {service name: service client instance}
        self._srvs = {}  # {service name: service server instance}
        self._reqs = {}  # {service name: service request instance}
        self._exps = {}  # {service name: expected request or response data dictionary}

        self.init_local_node()


    def setUp(self):
        """Opens service servers and clients."""
        super().setUp()
        self.run_test_node("--no-topics")

        for opts in testnode.DEFAULTS.get("service", {}).values():
            if "action" not in opts or "service" != opts["action"].get("category"):
                continue  # for opts
            action = opts["action"]
            handler = functools.partial(self.on_service, action["name"])
            logger.info("Opening service server at %r as %s.", action["name"], action["type"])
            self._srvs[action["name"]] = rosros.create_service(action["name"], action["type"], handler)
            logger.info("Opening service client to %r as %s.", opts["name"], opts["type"])
            self._clis[opts["name"]] = rosros.create_client(opts["name"], opts["type"])
            self._exps[opts["name"]] = opts.get("value", {})
            self._exps[action["name"]] = action.get("value", {})

        rosros.start_spin()


    def on_service(self, name, req):
        """Handler for service call, registers request object."""
        logger.info("Incoming service call to %r with %s.", name, req)
        self._reqs.setdefault(name, []).append(req)
        return {}


    def test_services(self):
        """Tests invoking and providing services."""
        resps = {}  # {service name: received response object}
        logger.debug("Starting test.")
        self.assertTrue(self._clis, "No services configured.")

        logger.info("Waiting for %s services.", len(self._clis))
        for name, cli in self._clis.items():
            logger.info("Waiting for service %r.", name)
            cli.wait_for_service(5)

        logger.info("Verifying service client service_is_ready().")
        for name, cli in self._clis.items():
            self.assertTrue(cli.service_is_ready(), "Unexpected result from service_is_ready().")

        for name, cli in self._clis.items():
            for req in (cli.request_class(), rosros.api.message_to_dict(cli.request_class()), None):
                logger.info("Invoking service at %r (%s).", name, "" if req is None else req)
                resp = cli() if req is None else cli(req)
                resps.setdefault(name, []).append(resp)
                time.sleep(1)  # ROS2 services can deadlock if callbacks are launched

        logger.info("Waiting for actions in %s service clients.", len(self._clis))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline \
        and not all(self._reqs.get(name) for name in self._srvs):
            rosros.spin_once(1)

        logger.info("Verifying received responses for %s service clients.", len(self._clis))
        for name in self._clis:
            logger.info("Verifying received responses for service %r.", name)
            with self.subTest(name):
                self.assertTrue(resps.get(name), "Expected response not received in service %r." % name)
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



if "__main__" == __name__:
    TestServices.run_rostest()
