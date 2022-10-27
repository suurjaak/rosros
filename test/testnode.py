#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test node to test against, uses rosros API to provide topics and services.

Supported command-line arguments:

--no-topics              ignore publioh and subscribe configuration
--latch-subscribe TOPIC  open subscription to topic with QoS durability=TRANSIENT_LOCAL

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     12.02.2022
@modified    27.10.2022
------------------------------------------------------------------------------
"""
import copy
import functools
import logging
import os
import sys
import threading
import time

import rosros
import rosros.util

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from test import testbase

logger = logging.getLogger()


## Default parameters for pubs-subs-services-clients
DEFAULTS = {
    "publish": {
        "boolout": {
            "name": "/test/topic/out/std_msgs/Bool",
            "type":  "std_msgs/Bool",
            "qos":   {},
            # "queue_size": 10,  # Optional shorthand for qos.depth
        },
        "headerout": {
            "name": "/test/topic/out/std_msgs/Header",
            "type":  "std_msgs/Header",
            "qos":   {},
            # "queue_size": 10,  # Optional shorthand for qos.depth
        },
    },
    "subscribe": {
        "boolin": {
            "name": "/test/topic/in/std_msgs/Bool",
            "type":  "std_msgs/Bool",
            "qos":   {},
            # "queue_size": 10,  # Optional shorthand for qos.depth
            "action": {
                "category": "publish",
                "name":     "/test/topic/out/std_msgs/Bool",
                "type":     "std_msgs/Bool",
                "value": {
                    "data": True,
                },
            },
        },
        "headerin": {
            "name": "/test/topic/in/std_msgs/Header",
            "type":  "std_msgs/Header",
            "qos":   {},
            # "queue_size": 10,  # Optional shorthand for qos.depth
            "action": {
                "category": "publish",
                "name":     "/test/topic/out/std_msgs/Header",
                "type":     "std_msgs/Header",
                "value": {
                    "frame_id": "headerout",
                },
            },
        },
    },
    "service": {
        "setboolin": {
            "name": "/test/service/in/std_srvs/SetBool",
            "type": "std_srvs/SetBool",
            "value": {
                "success": True,
                "message": "setboolin",
            },
            "action": {
                "category": "service",
                "name":     "/test/service/out/std_srvs/SetBool",
                "type":     "std_srvs/SetBool",
                "value": {
                    "data": True,
                },
            },
        },
    },
    "client": {
        "setboolout": {
            "name": "/test/service/out/std_srvs/SetBool",
            "type": "std_srvs/SetBool",
        },
    },
}



class TestNode():
    """
    Opens publishers and subscribers, service servers and service clients.

    Subscriptions and service servers can make configured actions:
    publishing to a topic, or calling a service.
    """

    ## Seconds to keep node running if no connections made
    STANDALONE_LIFETIME = 10

    ## Node default name
    NAME = "testnode"

    ## Node namespace
    NAMESPACE = "/tests"


    def __init__(self, params, name=NAME, namespace=NAMESPACE):
        """
        @param   params  node parameters, as {
                             "publish":   {label: {name, type, ?qos, ?queue_size}},
                             "subscribe": {label: {name, type, ?qos, ?queue_size,
                                           ?action: {category, name, type, value}}},
                             "service":   {label: {name, type, ?value}},
                             "client":    {label: {name, type}}, }
        """
        self._pubs = {}
        self._subs = {}
        self._srvs = {}
        self._clis = {}
        self._opts = {}
        self._ts_start = time.monotonic()

        testbase.init_logging(name)
        rosros.init_node(name, namespace=namespace, multithreaded=True, reentrant=True)
        params = rosros.init_params(params)
        self._opts.update(params)
        self.create_publishers()

        deadline = time.monotonic() + 5
        logger.info("Waiting for subscribers in %s topics.", len(self._pubs))
        while time.monotonic() < deadline \
        and sum(bool(x.get_num_connections()) for x in self._pubs.values()) < len(self._pubs):
            rosros.spin_once(1)

        self.create_subscribers()
        self.create_services()
        rosros.create_timer(1, self.on_watchdog)


    def create_publishers(self):
        """Opens publishers to configured topics."""
        for props in self._opts.get("publish", {}).values():
            topic, typename, qos = props["name"], props["type"], props.get("qos", {})
            if "queue_size" in props: qos.setdefault("depth", props["queue_size"])
            logger.debug("Opening publisher to %r as %s.", topic, typename)
            self._pubs[topic] = rosros.create_publisher(topic, typename, **qos)


    def create_subscribers(self):
        """Opens subscribers to configured topics."""
        for props in self._opts.get("subscribe", {}).values():
            topic, typename, qos = props["name"], props["type"], props.get("qos", {})
            if "queue_size" in props: qos.setdefault("depth", props["queue_size"])
            logger.debug("Opening subscriber to %r as %s.", topic, typename)
            handler = functools.partial(self.on_message, topic)
            self._subs[topic] = rosros.create_subscriber(topic, typename, handler, **qos)


    def create_services(self):
        """Opens service servers and clients as configured."""
        for props in self._opts.get("service", {}).values():
            name, typename = props["name"], props["type"]
            logger.debug("Opening service %r as %s.", name, typename)
            handler = functools.partial(self.on_service, name)
            self._srvs[name] = rosros.create_service(name, typename, handler)
        for props in self._opts.get("client", {}).values():
            name, typename = props["name"], props["type"]
            logger.debug("Opening service client to %r as %s.", name, typename)
            self._clis[name] = rosros.create_client(name, typename)


    def on_message(self, topic, msg):
        """Handler for receiving a message, carries out configured action, if any."""
        logger.debug("Received message in topic %r: %s.", topic, msg)
        props = next((x for x in self._opts["subscribe"].values() if x["name"] == topic), {})
        if props.get("action"):
            self.do_action(props["action"], "incoming message", topic, msg)


    def on_service(self, service, req):
        """Handler for receiving a service request, carries out configured action, if any."""
        logger.debug("Received service request in %r: %s.", service, req)
        props = next((x for x in self._opts["service"].values() if x["name"] == service), {})
        if props.get("action"):
            logger.debug("Starting one-shot timer to perform service action.")
            callback = lambda: self.do_action(props["action"], "service request", service, req)
            rosros.create_timer(0, callback, oneshot=True, immediate=True)
        return props.get("value", {})


    def on_watchdog(self):
        """Exits process if no connections after STANDALONE_LIFETIME seconds."""
        if time.monotonic() - self._ts_start < self.STANDALONE_LIFETIME: return

        if not any(x.get_num_connections() for x in self._pubs.values()) \
        and not any(x.get_num_connections() for x in self._subs.values()) \
        and not any(x.service_is_ready() for x in self._clis.values()):
            logger.info("Shutting down: no connection for %s seconds.", self.STANDALONE_LIFETIME)
            rosros.shutdown()
            sys.exit()


    def do_action(self, action, event, source, arg=None):
        """
        Carries out action in a callback thread, in response to a source event.

        @param   action  {"category", "name", ?"value"}}
        @param   event   triggering event label like "incoming message"
        @param   source  triggering event source like subscribed topic name
        @param   arg     input triggering the action, like ROS message or service request
        """
        target, args, kwargs = None, (), {}
        if "publish" == action["category"]:
            logger.debug("Executing reaction to %r (source %r): publish to %r.", event, source, action["name"])
            pub = self._pubs[action["name"]]
            target, kwargs = pub.publish, action.get("value", {})
        if "service" == action["category"]:
            logger.debug("Executing reaction to %r (source %r): call service %r.", event, source, action["name"])
            cli = self._clis[action["name"]]
            target, kwargs = cli.call, action.get("value", {})
        if target:
            threading.Thread(target=target, args=args, kwargs=kwargs).start()



if "__main__" == __name__:
    params = copy.deepcopy(DEFAULTS)
    if "--latch-subscribe" in sys.argv:
        topic = sys.argv[sys.argv.index("--latch-subscribe") + 1]
        for opts in params.get("subscribe", {}).values():
            if opts["name"] == topic:
                opts["qos"].update(durability=1)  # DurabilityPolicy.TRANSIENT_LOCAL
    if "--no-topics" in sys.argv:
        params.pop("publish", None)
        params.pop("subscribe", None)
    try:
        TestNode(params=params)
        rosros.spin()
    except KeyboardInterrupt: pass
