# -*- coding: utf-8 -*-
"""
Base class for ROS tests.

------------------------------------------------------------------------------
This file is part of rosros - grep for ROS bag files and live topics.
Released under the BSD License.

@author      Erki Suurjaak
@created     11.04.2022
@modified    25.06.2022
------------------------------------------------------------------------------
"""
import contextlib
import logging
import os
import subprocess
import sys
import threading
import time
import traceback
import unittest

if os.getenv("ROS_VERSION") != "2":
    import rospy
    import rostest
    ROS1, ROS2 = True, False
else:
    ROS1, ROS2 = False, True

import rosros


logger = logging.getLogger()


def init_logging(name):
    """Initializes logging."""
    fmt = "[%%(levelname)s]\t[%%(created).06f] [%s] %%(message)s" % name
    logging.basicConfig(level=logging.DEBUG, format=fmt, stream=sys.stdout)
    logger.setLevel(logging.DEBUG)


class TestBase(unittest.TestCase):
    """Tests grepping from a source to a sink, with prepared test bags."""

    ## Test name used in flow logging, overriden in subclasses
    NAME = ""

    ## Node namespace
    NAMESPACE = None

    ## Command for launching testnode
    NODE_CMD = "python3 testnode.py"

    ## Seconds to wait before force-exiting test
    TIMEOUT = 60


    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.maxDiff = None  # Full diff on assert failure
        try: unittest.util._MAX_LENGTH = 100000
        except Exception: pass

        self._proc = None   # subprocess.Popen for testnode

        init_logging(self.NAME)
        threading.Thread(target=self.ensure_exit, daemon=True).start()

    def setUp(self):
        logger.debug("Setting up test.")

    def tearDown(self):
        """Shuts down ROS, if any."""
        logger.debug("Tearing down test.")
        self.shutdown_test_node()
        try: rosros.shutdown()
        except Exception:
            logger.exception("Error shutting down ROS.")
            raise

    def init_local_node(self):
        """Creates ROS1 or ROS2 node."""
        rosros.init_node(self.NAME, namespace=self.NAMESPACE)
        self.add_logging()

    def add_logging(self):
        """Adds custom handler for ROS1 test output."""
        if ROS1:
            logger.addHandler(Ros1LogHandler(self.NAME))
            logger.setLevel(logging.DEBUG)

    def ensure_exit(self):
        """Exits after self.TIMEOUT seconds."""
        time.sleep(self.TIMEOUT)
        logger.warning("Force-exiting test after %s seconds.", self.TIMEOUT)
        try: self.tearDown()
        except Exception: logger.exception("Error tearing down test.")
        os._exit(1)

    def run_test_node(self):
        """Launches testnode in a subprocess."""
        if self._proc: return
        logger.debug("Executing %r.", self.NODE_CMD)
        self._proc = subprocess.Popen(self.NODE_CMD, shell=True, cwd=os.path.dirname(__file__))

    def shutdown_test_node(self):
        """Shuts down testnode in ROS2, if any."""
        if not self._proc: return
        args = dict(shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, timeout=5)
        for pid in self.get_process_children(self._proc.pid):
            with contextlib.suppress(Exception):
                subprocess.call("kill -9 %s" % pid, **args)
        with contextlib.suppress(Exception):
            subprocess.call("kill -9 %s" % self._proc.pid, **args)
        self._proc = None

    @contextlib.contextmanager
    def subTest(self, msg=None, **params):
        """Overrides `TestCase.subTest` to log any exceptions."""
        with super().subTest(msg, **params):
            try: yield
            except Exception:
                logger.info("Exception in subTest %r.\n%s", msg, traceback.format_exc())
                raise

    @classmethod
    def get_process_children(cls, pid):
        """Returns a list of child process IDs, in recursive bottom-up order."""
        result, lookups = [], [pid]
        CMD = "ps --no-header -opid --ppid %s"
        args = dict(shell=True, stderr=subprocess.DEVNULL, timeout=10)
        while lookups:
            output = ""
            with contextlib.suppress(Exception):
                output = subprocess.check_output(CMD % lookups.pop(0), **args).decode()
            for line in filter(bool, output.splitlines()):
                with contextlib.suppress(ValueError):
                    result.insert(0, int(line)), lookups.insert(0, int(line))
        return result

    @classmethod
    def run_rostest(cls):
        """Runs rostest if ROS1."""
        if ROS1: rostest.rosrun("rosros", cls.NAME, cls)


class Ros1LogHandler(logging.Handler):
    """Logging handler that forwards logging messages to rospy.logwarn or higher."""

    def __init__(self, name, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__name = name

    def emit(self, record):
        """Invokes rospy.logwarn or logerr (only warn or higher gets rostest output)."""
        if "rosout" == record.name or record.name.startswith("rospy"):
            return  # Skip rospy internal logging

        try: text = record.msg % record.args if record.args else record.msg
        except Exception: text = record.msg
        text = "[%s] %s" % (self.__name, text)
        (rospy.logerr if record.levelno >= logging.ERROR else rospy.logwarn)(text)
