# -*- coding: utf-8 -*-
"""
ROS1 core interface.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     11.02.2022
@modified    28.10.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.ros1
import collections
import datetime
import decimal
import functools
import inspect
import io
import logging
import os
import re
import shutil
import threading
import time
import traceback

import genpy
import rosbag
import roslib
import rospy
import rospy.core
import rospy.names
import rospy.rostime
import rosservice

from . import parsing
from . import patch
from . import rclify
from . import util


## `rospy.AnyMsg`
AnyMsg = rospy.AnyMsg

## ROS1 time/duration types
ROS_TIME_TYPES = ["time", "duration"]

## ROS1 time/duration types mapped to type names
ROS_TIME_CLASSES = {rospy.Time: "time", rospy.Duration: "duration"}

## Mapping between ROS type aliases and real types, like {"byte": "int8"}
ROS_ALIAS_TYPES = {"byte": "int8", "char": "uint8"}

## Separator char between ROS1 parameter namespace parts
PARAM_SEPARATOR = "/"

## Prefix for "private" names, auto-namespaced under current namespace
PRIVATE_PREFIX = "~"

## ROS Python module family, "rospy"
FAMILY = "rospy"

## Map rospy log level constants to Python logging level constants
PY_LOG_LEVEL_TO_ROSPY_LEVEL = {
    logging.DEBUG:  1,
    logging.INFO:   2,
    logging.WARN:   4,
    logging.ERROR:  8,
    logging.FATAL: 16,
}

## rospy.MasterProxy instance
MASTER = None

## `threading.Thread` for start_spin()
SPINNER = None


logger = util.ThrottledLogger(logging.getLogger(__name__))



class ROSLogHandler(logging.Handler):
    """Logging handler that forwards logging messages to ROS1 logger."""

    def __init__(self):
        super().__init__()
        self._logger = logging.getLogger("rosout")  # logging.Logger instance


    def emit(self, record):
        """Adds message to ROS1 logger."""
        if not self._logger.isEnabledFor(record.levelno):
            return
        try:
            text = record.msg % record.args if record.args else record.msg
        except Exception:
            text = record.msg
        if record.exc_info:
            text += "\n\n" + "".join(traceback.format_exception(*record.exc_info))
        self._logger.log(record.levelno, text)



class Bag(rosbag.Bag):
    """
    ROS1 bag reader and writer.
    
    Extends `rosbag.Bag` with more conveniences, and smooths over the rosbag bug
    of yielding messages of wrong type, if message types in different topics
    have different packages but identical fields and hashes.

    Does **not** smooth over the rosbag bug of writing different types to one topic.

    rosbag does allow writing messages of different types to one topic,
    just like live ROS topics can have multiple message types published
    to one topic. And their serialized bytes will actually be in the bag,
    but rosbag will only register the first type for this topic (unless it is
    explicitly given another connection header with metadata on the other type).

    All messages yielded will be deserialized by rosbag as that first type,
    and whether reading will raise an exception or not depends on whether 
    the other type has enough bytes to be deserialized as that first type.
    """

    ## Default extension for bag files
    EXTENSION = ".bag"

    # {(typename, typehash): message type class}
    __TYPES    = {}

    ## {(typename, typehash): type definition text}
    __TYPEDEFS = {}

    # {(typename, typehash): whether subtype definitions parsed}
    __PARSEDS = {}


    def __init__(self, f, mode="r",
                 compression=rosbag.Compression.NONE, chunk_threshold=768 * 1024,
                 allow_unindexed=False, options=None, skip_index=False):
        """
        Opens a ROS1 bag file.

        @param   f                bag file path to open
        @param   mode             mode to open bag in, one of "r", "w", "a"
        @param   compression      bag write compression mode, one of Compression
        @param   chunk_threshold  minimum number of uncompressed bytes per chunk to write
        @param   allow_unindexed  allow opening unindexed bags
        @param   options          optional dict with values for "compression" and "chunk_threshold"
        @param   skip_index       skip reading the connection index records on open
        """
        super().__init__(f, mode, compression, chunk_threshold,
                         allow_unindexed, options, skip_index)
        self.__topics = {}  # {(topic, typename, typehash): message count}
        self.__populate_meta()


    def get_message_definition(self, msg_or_type):
        """Returns ROS1 message type definition full text from bag, including subtype definitions."""
        if is_ros_message(msg_or_type):
            return self.__TYPEDEFS.get((msg_or_type._type, msg_or_type._md5sum))
        typename = msg_or_type
        return next((d for (n, h), d in self.__TYPEDEFS.items() if n == typename), None)


    def get_message_class(self, typename, typehash=None):
        """
        Returns ROS1 message class for typename, or None if unknown type.

        Generates class dynamically if not already generated.

        @param   typehash  message type definition hash, if any
        """
        self.__ensure_typedef(typename, typehash)
        typehash = typehash or next((h for n, h in self.__TYPEDEFS if n == typename), None)
        typekey = (typename, typehash)
        if typekey not in self.__TYPES and typekey in self.__TYPEDEFS:
            for n, c in genpy.dynamic.generate_dynamic(typename, self.__TYPEDEFS[typekey]).items():
                self.__TYPES[(n, c._md5sum)] = c
        return self.__TYPES.get(typekey)


    def get_message_type_hash(self, msg_or_type):
        """Returns ROS1 message type MD5 hash."""
        if is_ros_message(msg_or_type): return msg_or_type._md5sum
        typename = msg_or_type
        typehash = next((h for n, h in self.__TYPEDEFS if n == typename), None)
        if not typehash:
            self.__ensure_typedef(typename)
            typehash = next((h for n, h in self.__TYPEDEFS if n == typename), None)
        return typehash
 

    def get_qoses(self, topic, typename):
        """Returns None (ROS2 bag API conformity stand-in)."""
        return None


    def get_topic_info(self, *_, **__):
        """Returns topic and message type metainfo as {(topic, typename, typehash): count}."""
        return dict(self.__topics)


    def read_messages(self, topics=None, start_time=None, end_time=None, connection_filter=None, raw=False):
        """
        Yields messages from the bag in chronological order.
        
        @param   topics             list of topics or a single topic.
                                    If an empty list is given, all topics will be read.
        @param   start_time         earliest timestamp of messages to return,
                                    as `rospy.Time` or convertible
                                    (int/float/duration/datetime/decimal)
        @param   end_time           latest timestamp of messages to return
                                    as `rospy.Time` or convertible
                                    (int/float/duration/datetime/decimal)
        @param   connection_filter  function to filter connections to include
        @param   raw                if True, then returned messages are tuples of
                                    (typename, bytes, md5sum, typeclass)
                                    or (typename, bytes, md5sum, position, typeclass),
                                    depending on file format version
        @return                     generator of BagMessage(topic, message, timestamp) namedtuples
        """
        hashtypes = {}
        for n, h in self.__TYPEDEFS: hashtypes.setdefault(h, []).append(n)
        read_topics = topics if isinstance(topics, list) else [topics] if topics else None
        dupes = {t: (n, h) for t, n, h in self.__topics
                 if (read_topics is None or t in read_topics) and len(hashtypes.get(h, [])) > 1}

        kwargs = dict(topics=topics, start_time=to_time(start_time), end_time=to_time(end_time),
                      connection_filter=connection_filter, raw=raw)
        if not dupes:
            for topic, msg, stamp in super().read_messages(**kwargs):
                yield rosbag.bag.BagMessage(topic, msg, stamp)
            return

        for topic, msg, stamp in super().read_messages(**kwargs):
            # Workaround for rosbag bug of using wrong type for identical type hashes
            if topic in dupes:
                typename, typehash = (msg[0], msg[2]) if raw else (msg._type, msg._md5sum)
                if dupes[topic] != (typename, typehash):
                    if raw:
                        msg = msg[:-1] + (self.get_message_class(typename, typehash), )
                    else:
                        msg = self.__convert_message(msg, *dupes[topic])
            yield rosbag.bag.BagMessage(topic, msg, stamp)


    def write(self, topic, msg, t=None, raw=False, connection_header=None, **__):
        """
        Writes a message to the bag.

        @param    topic              name of topic
        @param    msg                ROS message to write, or tuple if raw
        @param    t                  message timestamp if not using current wall time,
                                     as `rospy.Time` or convertible
                                     (int/float/duration/datetime/decimal)
        @param    raw                if true, msg is expected
                                     as (typename, bytes, typehash, typeclass)
                                     or (typename, bytes, typehash, position, typeclass)
        @param    connection_header  custom connection header dict if any,
                                     as {"topic", "type", "md5sum", "message_definition"}
        """
        return super().write(topic, msg, to_time(t), raw, connection_header)


    def __convert_message(self, msg, typename2, typehash2=None):
        """Returns message converted to given type; fields must match."""
        msg2 = self.get_message_class(typename2, typehash2)()
        fields2 = get_message_fields(msg2)
        for fname, ftypename in get_message_fields(msg).items():
            v1 = v2 = getattr(msg, fname)
            if ftypename != fields2.get(fname, ftypename):
                v2 = self.__convert_message(v1, fields2[fname])
            setattr(msg2, fname, v2)
        return msg2


    def __populate_meta(self):
        """Populates topics and message type definitions and hashes."""
        result = collections.Counter()  # {(topic, typename, typehash): count}
        counts = collections.Counter()  # {connection ID: count}
        for c in self._chunks:
            for c_id, count in c.connection_counts.items():
                counts[c_id] += count
        for c in self._connections.values():
            result[(c.topic, c.datatype, c.md5sum)] += counts[c.id]
            self.__TYPEDEFS[(c.datatype, c.md5sum)] = c.msg_def
        self.__topics = dict(result)


    def __ensure_typedef(self, typename, typehash=None):
        """Parses subtype definition from any full definition where available, if not loaded."""
        typehash = typehash or next((h for n, h in self.__TYPEDEFS if n == typename), None)
        typekey = (typename, typehash)
        if typekey not in self.__TYPEDEFS:
            for (roottype, roothash), rootdef in list(self.__TYPEDEFS.items()):
                rootkey = (roottype, roothash)
                if self.__PARSEDS.get(rootkey): continue  # for (roottype, roothash)

                subdefs = tuple(parsing.parse_definition_subtypes(rootdef).items())
                subhashes = {n: parsing.calculate_definition_hash(n, d, subdefs)
                             for n, d in subdefs}
                self.__TYPEDEFS.update(((n, subhashes[n]), d) for n, d in subdefs)
                self.__PARSEDS.update(((n, h), True) for n, h in subhashes.items())
                self.__PARSEDS[rootkey] = True
                if typekey in self.__TYPEDEFS:
                    break  # for (roottype, roothash)
            self.__TYPEDEFS.setdefault(typekey, "")


    @staticmethod
    def reindex_file(f):
        """
        Reindexes bag file on disk.

        Makes a temporary copy in file directory.
        """
        with rosbag.Bag(f, allow_unindexed=True, skip_index=True) as inbag:
            inplace = (inbag.version > 102)

        f2 = util.unique_path("%s.orig%s") % os.path.splitext(f)
        shutil.copy(f, f2)
        inbag, outbag = None, None
        try:
            inbag  = rosbag.Bag(f2, allow_unindexed=True) if not inplace else None
            outbag = rosbag.Bag(f, mode="a" if inplace else "w", allow_unindexed=True)
            # v102: build index from inbag, write all messages to outbag.
            # Later versions: re-build index in outbag file in-place.
            for _ in (outbag if inplace else inbag).reindex(): pass
            if not inplace:
                for (topic, msg, t, header) in inbag.read_messages(return_connection_header=True):
                    outbag.write(topic, msg, t, connection_header=header)
        except BaseException:  # Ensure steady state even on KeyboardInterrupt/SystemExit
            inbag and inbag.close()
            outbag and outbag.close()
            shutil.move(f2, f)  # Restore original from temporary copy
            raise
        inbag and inbag.close()
        outbag and outbag.close()
        os.remove(f2)  # Drop temporary copy



class Mutex:
    """Container for local mutexes."""

    ## Mutex for `spin()`
    SPIN      = threading.RLock()

    ## Mutex for `start_spin()`
    SPIN_START = threading.RLock()



def init_node(name, args=None, namespace=None, anonymous=False, log_level=None, enable_rosout=True):
    """
    Initializes rospy and creates ROS1 node.

    @param   name           node name, without namespace
    @param   args           list of command-line arguments for the node
    @param   namespace      node namespace override
    @param   anonymous      whether to auto-generate a unique name for the node,
                            using the given name as base
    @param   log_level      level to set for ROS logging
                            (name like "DEBUG" or one of `logging` constants like `logging.DEBUG`)
    @param   enable_rosout  `False` to suppress auto-publication of rosout
    """
    global MASTER, logger
    if MASTER: return

    if namespace and namespace != "/":
        os.environ["ROS_NAMESPACE"] = namespace
        rospy.names._set_caller_id(util.namejoin(namespace, name))

    ros_level = None
    if log_level is not None:
        if isinstance(log_level, str): log_level = logging.getLevelName(log_level)
        ros_level = PY_LOG_LEVEL_TO_ROSPY_LEVEL.get(log_level)

    patch.patch_ros1()
    logger.debug("Initializing ROS node %r.", name)
    rospy.init_node(name, args, anonymous=anonymous, log_level=ros_level,
                    disable_rosout=not enable_rosout, disable_signals=True)
    MASTER = rospy.client.get_master()

    if not any(isinstance(x, ROSLogHandler) for x in logger.handlers):
        logger.addHandler(ROSLogHandler())
    if log_level is not None:
        logger.setLevel(log_level)


def register_init():
    """Informs `rosros` of ROS1 having been initialized outside of `init_node()`."""
    global MASTER
    if MASTER: return

    patch.patch_ros1()
    MASTER = rospy.client.get_master()
    if not any(isinstance(x, ROSLogHandler) for x in logger.handlers):
        logger.addHandler(ROSLogHandler())


def init_params(defaults=None, **defaultkws):
    """
    Declares all parameters on node from defaults dictionary.

    @param   defaults    nested dictionary with suitable keys and values for ROS
                         (keys must be valid namespace names,
                          list values must not contain nested values)
    @param   defaultkws  parameters as key=value
    @return              full nested parameters dictionary, combined from given defaults
                         and externally set parameters on the node
    """
    result = {}
    stack = list(util.merge_dicts(defaults or {}, defaultkws).items())
    while stack:
        k, v = stack.pop()
        if isinstance(v, dict):
            if not v:
                util.set_value(result, k.split(PARAM_SEPARATOR), v)
            stack.extend(("%s%s%s" % (k, PARAM_SEPARATOR, k2), v2) for k2, v2 in v.items())
        else:
            k = "~" + format_param_name(k)
            if not rospy.has_param(k):
                rospy.set_param(k, v)

    prefix = get_node_name().rstrip(PARAM_SEPARATOR) + PARAM_SEPARATOR
    for path in rospy.get_param_names():
        if path.startswith(prefix):
            path = path[len(prefix):]
            util.set_value(result, [x for x in path.split(PARAM_SEPARATOR) if x],
                           rospy.get_param("~" + path))
    logger.debug("Initialized node parameters %r.", result)
    return result


def has_param(name):
    """
    Returns whether the parameter exists.

    @param   name  full name of the parameter in node namespace
    """
    return rospy.has_param("~" + format_param_name(name))


def get_param(name, default=None, autoset=True):
    """
    Returns parameter value from current ROS1 node.

    @param   name      full name of the parameter in node namespace
    @param   default   optional default to return if parameter unknown
    @param   autoset   set default value to node parameter if unknown
    @return            parameter value, or default if parameter was unknown

    @throws  KeyError  if parameter not set and default not given
    """
    name = "~" + format_param_name(name)
    if autoset and default is not None and not rospy.has_param(name):
        rospy.set_param(name, default)
    return rospy.get_param(name, default)


def get_param_names():
    """Returns the names of all current ROS1 node parameters."""
    result = []
    prefix = get_node_name().rstrip(PARAM_SEPARATOR) + PARAM_SEPARATOR
    for path in rospy.get_param_names():
        if path.startswith(prefix): result.append(path[len(prefix):])
    return result


def get_params(nested=True):
    """
    Returns the current ROS1 node parameters, by default as nested dictionary.

    @param   nested  return a nested dictionary,
                     like `{"my": {"name": 1}}` vs {"my.name": 1}
    """
    result = {}
    prefix = get_node_name().rstrip(PARAM_SEPARATOR) + PARAM_SEPARATOR
    for path in rospy.get_param_names():
        if path.startswith(prefix):
            path = path[len(prefix):]
            key = [x for x in path.split(PARAM_SEPARATOR) if x] if nested else path
            util.set_value(result, key, rospy.get_param("~" + path))
    return result


def set_param(name, value):
    """
    Sets a parameter on the node.

    @param   name   full name of the parameter in node namespace
    @param   value  parameter value to set
    @return         the set value
    """
    rospy.set_param("~" + format_param_name(name), value)
    return value


def delete_param(name):
    """
    Deletes parameter from the node.

    @param   name      full name of the parameter in node namespace
    @throws  KeyError  if parameter not set
    """
    rospy.delete_param("~" + format_param_name(name))


def ok():
    """Returns whether ROS1 has been initialized and is not shut down."""
    return rospy.core.is_initialized() and \
           not (rospy.is_shutdown() or rospy.core.is_shutdown_requested())


def start_spin():
    """
    Sets ROS1 spinning forever in a background thread.

    Does nothing if spinning has already been started.
    """
    def do_spin():
        global SPINNER
        try:
            while SPINNER and ok(): spin_once(0.5)
        finally: SPINNER = None

    global SPINNER
    with Mutex.SPIN_START:
        if SPINNER or Mutex.SPIN._is_owned(): return
        SPINNER = threading.Thread(target=do_spin)
        SPINNER.start()


def spin():
    """Spins ROS1 forever."""
    global SPINNER

    if SPINNER:
        try: SPINNER.join()  # Wait on background thread instead
        except KeyboardInterrupt: SPINNER = None  # Signal thread to end
        return

    if Mutex.SPIN._is_owned():  # spin() started from another thread
        with Mutex.SPIN: return  # Wait on mutex instead
    with Mutex.SPIN:
        rospy.spin()


def spin_once(timeout=None):
    """
    Waits until timeout, or forever if timeout None or negative.

    @param  timeout  time to sleep, as seconds or ROS duration;
                     None or <0 waits forever
    """
    rospy.rostime.wallsleep(2**31 - 1 if timeout is None else to_sec(timeout))


def spin_until_future_complete(future, timeout=None):
    """
    Spins ROS1 until future complete or timeout reached or ROS shut down.

    @param  future   object with `asyncio.Future`-conforming interface to complete
                     (will not be awaited with `async`)
    @param  timeout  time to wait, as seconds or ROS1 duration
    """
    if future.done():
        return
    timeout = to_sec(timeout)
    if timeout == 0.0:
        if not future.done():
            spin_once(1E-9)
        return

    now = time.monotonic()
    deadline = (now + timeout) if timeout is not None and timeout >= 0 else -1
    while ok() and (deadline < 0 or now < deadline) and not future.done():
        spin_once(0.1)
        now = time.monotonic()
    if not future.done() and not ok():
        future.cancel("ROS shut down")


def shutdown():
    """Sends the shutdown signal to rospy."""
    rospy.signal_shutdown(reason="")


def create_client(service, cls_or_typename):
    """
    Returns a ROS1 service client instance, for invoking a service.

    @param   service          name of service to invoke
    @param   cls_or_typename  ROS1 service class object like `std_srvs.srv.SetBool`
                              or service type name like `"std_srvs/SetBool"
    @return                   `rospy.ServiceProxy`,
                              will have `call_async()` returning a future
    """
    return rospy.ServiceProxy(service, get_message_class(cls_or_typename) )


def create_service(service, cls_or_typename, callback):
    """
    Returns a ROS1 service server instance, for providing a service.

    @param   service          name of service to provide
    @param   cls_or_typename  ROS1 service class object like `std_srvs.srv.SetBool`
                              or service type name like `"std_srvs/SetBool"
    @param   callback         callback function, invoked with (req, resp) or (req)
    @return                   `rospy.Service`
    """
    cls = get_message_class(cls_or_typename)
    arity = util.get_arity(callback)
    if arity in (-1, 2):
        mycallback = functools.partial((lambda f, c, r: f(r, c._response_class())), callback, cls)
    else:
        mycallback = util.wrap_arity(callback) if arity != 1 else callback
    return rospy.Service(service, cls, mycallback)


def create_publisher(topic, cls_or_typename, latch=False, queue_size=0):
    """
    Returns a ROS1 publisher instance.

    @param   topic            name of topic to open
    @param   cls_or_typename  ROS1 message class object like `std_msgs.msg.Bool`
                              or message type name like "std_msgs/Bool"
    @param   latch            provide last published message to new subscribers
    @param   queue_size       queue size of outgoing messages (0 or None: infinite)
    @return                   `rospy.Publisher`
    """
    cls = get_message_class(cls_or_typename)
    return rospy.Publisher(topic, cls, latch=latch, queue_size=queue_size or 0)


def create_subscriber(topic, cls_or_typename, callback, callback_args=None,
                      queue_size=0, raw=False):
    """
    Returns a ROS1 subscriber instance.

    @param   topic            name of topic to listen to
    @param   cls_or_typename  ROS1 message class object like `std_msgs.msg.Bool`
                              or message type name like "std_msgs/Bool"
    @param   callback         callback function, invoked with received message,
                              and with additional arguments if given
    @param   callback_args    additional arguments to pass to the callback, if any,
                              invoked as `callback(msg, callback_args)´
    @param   raw              make subscription with AnyMsg,
                              invoke callback with serialized message bytes
    @param   queue_size       queue size of incoming messages (0 or None: infinite)
    @return                   `rospy.Subscriber`
    """
    cls = get_message_class(cls_or_typename)
    anymsg = issubclass(cls, rospy.AnyMsg) and not raw  # Return AnyMsg if not explicitly raw
    if raw and not issubclass(cls, rospy.AnyMsg): cls = rospy.AnyMsg
    sub = rospy.Subscriber(topic, cls, callback, callback_args, queue_size=queue_size or None)
    sub._anymsg = anymsg
    return sub


def create_timer(period, callback, oneshot=False, immediate=False):
    """
    Returns a ROS1 timer instance.

    @param   period     desired period between callbacks, as seconds or ROS1 duration
    @param   callback   callback function invoked on timer, with no arguments
    @param   oneshot    whether to fire only once
    @param   immediate  whether to fire once immediately instead of waiting one period
    @return             `rospy.Timer`
    """
    period, mycallback = make_duration(period), lambda *_, **__: callback()
    if immediate:
        timer = rospy.Timer(rospy.Duration(nsecs=1), mycallback, oneshot=True)
    if not immediate or not oneshot:
        timer = rospy.Timer(period, mycallback, oneshot)
    return timer


def create_rate(frequency):
    """
    Returns a ROS1 rate instance, for sleeping at a fixed rate.

    @param   frequency  rate to sleep at, in Hz
    @return             `rospy.Rate`
    """
    return rospy.Rate(frequency)


def destroy_entity(item):
    """Closes the given publisher, subscriber, service client, or service server instance."""
    if isinstance(item, rospy.topics.Topic):
        item.unregister()
    elif isinstance(item, rospy.client.ServiceProxy):
        item.close()
    elif callable(getattr(item, "shutdown", None)):
        item.shutdown()  # Service or timer


def get_namespace():
    """Returns ROS1 node namespace."""
    return rospy.get_namespace()


def get_node_name():
    """Returns ROS1 node full name with namespace."""
    return rospy.get_name()


def get_nodes():
    """Returns all ROS1 nodes, as `[node full name, ]`."""
    # getSystemState() returns [[topic, [publishing node, ]], [..subs], [..services]]
    return sorted(set(l for s in MASTER.getSystemState()[-1] for _, ll in s for l in ll))


def get_topics():
    """Returns all available ROS1 topics, as `[(topic name, [type name, ]), ]`."""
    result = {}
    for n, t in MASTER.getTopicTypes()[-1]:
        result.setdefault(n, []).append(t)
    return sorted([n, sorted(map(canonical, tt))] for n, tt in result.items())


def get_services(node=None, namespace=None, include_types=True):
    """
    Returns all available ROS1 services, as `[(service name, [type name, ]), ]`.

    @param   node           full name of the node to return services for, if any
    @param   namespace      full or partial namespace to scope services from
    @param   include_types  if false, type names will be returned as an empty list
    """
    result = []
    for name in rosservice.get_service_list(node, namespace):
        if not include_types:
            result.append([name, []])
            continue  # for name
        try:
            typename = rosservice.get_service_type(name)
            if typename: result.append([name, [typename]])
        except Exception:
            logger.warning("Error retrieving service type for %s.", name, exc_info=True)
    return sorted(result)


def get_logger():
    """
    Returns `logging.Logger` for logging to ROS1 log handler.

    Logging methods on the logger (`debug()`, `info()`, etc) accept additional keyword arguments:
    - `__once__`:                whether to log only once from call site
    - `__throttle__`:            seconds to skip logging from call site for
    - `__throttle_identical__`:  whether to skip logging identical consecutive texts from call site
                                 (given log message excluding formatting arguments).
                                 Combines with `__throttle__` to skip duplicates for a period.
    """
    return logger


def get_rostime():
    """Returns current ROS1 time, as `rospy.Time`."""
    result = rospy.get_rostime()
    if patch.PATCHED_FULL and not rospy.rostime.is_wallclock() \
    and rclify.clock.ClockType.ROS_TIME != result.clock_type:
        patch.set_extra_attribute(result, "_clock_type", rclify.clock.ClockType.ROS_TIME)
    return result


def remap_name(name, namespace=None):
    """
    Returns the absolute remapped topic/service name if mapping exists.

    @param   name       name to seek exact remapping for
    @param   namespace  namespace to resolve relative and private names to,
                        by default current node namespace
    @return             remapped resolved name, or original if not mapped
    """
    return rospy.remap_name(name, namespace)


def resolve_name(name, namespace=None):
    """
    Returns absolute remapped name, namespaced under current node if relative or private.

    @param   namespace  namespace to use if not current node full name
    """
    namespace = namespace or get_node_name()
    if not name:  # empty string resolves to node namespace
        return util.namesplit(namespace)[0] + "/"

    # Discard multiple slashes
    name2 = ("/" if name.startswith("/") else "") + "/".join(filter(bool, name.split("/")))
    if name2.startswith("~"):  # private name
        name2 = util.namejoin(namespace, name2.lstrip("~"))
    elif not name2.startswith("/"):  # relative name
        name2 = util.namejoin(util.namesplit(namespace)[0], name2)

    return rospy.names.get_resolved_mappings().get(name2, name2)


# -------------------------------- GENERAL API --------------------------------


@util.memoize
def canonical(typename):
    """Returns "pkg/Type" for "pkg/subdir/Type"."""
    if typename.count("/") > 1:
        typename = "%s/%s" % tuple((x[0], x[-1]) for x in [typename.split("/")])[0]
    return typename


@util.memoize
def format_param_name(name):
    """Returns parameter name using "/" separator, and leading root or private sigils stripped."""
    return name.replace(".", PARAM_SEPARATOR).lstrip("~" + PARAM_SEPARATOR)


def get_message_class(msg_or_type):
    """
    Returns ROS1 message / service class object.

    @param   msg_or_type  full or canonical class name
                          like "std_msgs/Bool" or "std_srvs/SetBool" or "std_srvs/SetBoolRequest",
                          or class instance like `std_msgs.msg.Bool()`,
                          or class object like `std_msgs.msg.Bool`
    @return               ROS1 message / service class object, like `std_msgs.msg.Bool`
                          or `std_srvs.srv.SetBool` or `std_srvs.srv.SetBoolRequest`,
                          or None if not found
    """
    if is_ros_message(msg_or_type) or is_ros_service(msg_or_type):
        return msg_or_type if inspect.isclass(msg_or_type) else type(msg_or_type)

    typename, cls = canonical(msg_or_type), None
    if typename.startswith("%s/" % FAMILY):
        cls = next((c for c in ROS_TIME_CLASSES if get_message_type(c) == typename), None)
    try: cls = cls or roslib.message.get_message_class(typename)
    except Exception: cls = None
    if not cls:
        try: cls = roslib.message.get_service_class(typename)
        except Exception: cls = None
        if not cls and re.match(r".+_(Request|Response)$", typename):
            # Smooth over ROS1/ROS2 difference: ServiceRequest vs Service_Request
            typename = re.sub(r"(.+)_(Request|Response)$", r"\1\2", typename)
            try: cls = roslib.message.get_service_class(typename)
            except Exception: cls = None
    return cls


def get_message_definition(msg_or_type, full=True):
    """
    Returns ROS1 message or service request/response type definition text.

    Text will include subtype definitions by default.

    @param   msg_or_type  canonical or full class name like "std_msgs/Bool" or "std_msgs/msg/Bool",
                          or class instance like `std_msgs.msg.Bool()`,
                          or class object like `std_msgs.msg.Bool`
    @param   full         include definitions of nested types, separated by "\n---\nMSG: pkg/Type\n"
                          (ignored for service request/response types)
    @return               message type definition text
    """
    text = get_message_class(msg_or_type)._full_text
    if not full:  # Discard everything after a full line of "="
        text = re.sub(r"\n=+\n.+", "", text, flags=re.DOTALL)
    return text


def get_message_fields(val):
    """
    Returns {field name: field type name} if ROS1 message or service request/response, else {}.

    @param   val  ROS1 message or service request/response instance, or class object
    """
    names = getattr(val, "__slots__", [])
    if isinstance(val, tuple(ROS_TIME_CLASSES)):  # Empty __slots__
        names = genpy.TVal.__slots__
    return dict(zip(names, getattr(val, "_slot_types", [])))


def get_message_header(val):
    """
    Returns message `std_msgs/Header`-attribute if any, else `None`.

    @param   val  ROS message or service request/response instance
    """
    if is_ros_message(val) and val._has_header:
        for name, typename in get_message_fields(val):
            if "std_msgs/Header" == typename:
                return get_message_value(val, name)
    return None


def get_message_type(msg_or_cls):
    """
    Returns ROS1 message / service canonical type name, like "std_msgs/Header".

    Returns "*" for `AnyMsg`.

    @param   msg_or_cls  class instance like `std_msgs.msg.Bool()`,
                         or class object like `std_msgs.msg.Bool`
    @return              canonical name, or `None` if not ROS message / service
    """
    if is_ros_time(msg_or_cls):
        cls = msg_or_cls if inspect.isclass(msg_or_cls) else type(msg_or_cls)
        return "%s/%s" % (FAMILY, cls.__name__)
    return msg_or_cls._type if is_ros_message(msg_or_cls) else None


def get_message_type_hash(msg_or_type):
    """
    Returns ROS1 message / service type MD5 hash.

    @param   msg_or_type  full or canonical class name
                          like "std_msgs/Bool" or "std_srvs/SetBool" or "std_srvs/SetBoolRequest",
                          or class instance like `std_msgs.msg.Bool()`,
                          or class object like `std_msgs.msg.Bool`
    """
    return get_message_class(msg_or_type)._md5sum


def get_message_value(msg, name):
    """
    Returns object attribute value, with numeric arrays converted to lists.

    @param   message attribute name; may also be (nested, path) or "nested.path"
    """
    v, parent, k = util.get_nested(msg, name)
    if isinstance(v, bytes) and is_ros_message(parent) \
    and get_message_fields(parent)[k].startswith("uint8["):
        v = list(v)
    return v


def get_service_definition(srv_or_type):
    """
    Returns ROS1 service type definition text.

    @param   srv_or_type  canonical or full class name
                          like "std_srvs/SetBool" or "std_srvs/srv/SetBool",
                          or class instance like `std_srvs.srv.SetBool()`,
                          or class object like `std_srvs.srv.SetBool`
    @return               ROS service type definition text
    """
    cls = get_message_class(srv_or_type)
    reqcls, respcls = cls._request_class, cls._response_class
    reqtxt, resptxt = (get_message_definition(x, full=False) for x in (reqcls, respcls))
    return "%s\n---\n%s" % (reqtxt, resptxt)


def get_service_request_class(srv_or_type):
    """
    Returns ROS1 service request class object.

    @param   srv_or_type  canonical or full class name
                          like "std_srvs/SetBool" or "std_srvs/srv/SetBool",
                          or class instance like `std_srvs.srv.SetBool()`,
                          or class object like `std_srvs.srv.SetBool`
    @return               ROS1 service request class object, like `std_srvs.srv.SetBoolRequest`
    """
    return get_message_class(srv_or_type)._request_class


def get_service_response_class(srv_or_type):
    """
    Returns ROS1 service response class object.

    @param   srv_or_type  canonical or full class name
                          like "std_srvs/SetBool" or "std_srvs/srv/SetBool",
                          or class instance like `std_srvs.srv.SetBool()`,
                          or class object like `std_srvs.srv.SetBool`
    @return               ROS1 service response class object, like `std_srvs.srv.SetBoolResponse`
    """
    return get_message_class(srv_or_type)._response_class


def is_ros_message(val):
    """
    Returns whether value is a ROS1 message or service request/response class or instance.

    @param   val  like `std_msgs.msg.Bool()` or `std_srvs.srv.SetBoolRequest`
    @return       True if value is a ROS1 message or service request/response class or instance,
                  False otherwise
    """
    return (issubclass if inspect.isclass(val) else isinstance)(val, (genpy.Message, genpy.TVal))


def is_ros_service(val):
    """Returns whether value is a ROS1 service class object or instance."""
    STRFIELDS, MSGFIELDS = ('_md5sum', '_type'), ('_request_class', '_response_class')
    return all(isinstance(getattr(val, x, None), str) for x in STRFIELDS) and \
           all(is_ros_message(getattr(val, x, None)) for x in MSGFIELDS)


def is_ros_time(val):
    """Returns whether value is a ROS1 time/duration class or instance."""
    return issubclass(val, genpy.TVal) if inspect.isclass(val) else isinstance(val, genpy.TVal)


def make_duration(secs=0, nsecs=0):
    """Returns a ROS1 duration, as rospy.Duration."""
    return rospy.Duration(secs=secs, nsecs=nsecs)


def make_time(secs=0, nsecs=0):
    """Returns a ROS1 time, as rospy.Time."""
    return rospy.Time(secs=secs, nsecs=nsecs)


def serialize_message(msg):
    """Returns ROS1 message or service request/response as a serialized binary of `bytes()`."""
    buf = io.BytesIO()
    msg.serialize(buf)
    return buf.getvalue()


def deserialize_message(raw, cls_or_typename):
    """Returns ROS1 message or service request/response instantiated from serialized binary."""
    return get_message_class(cls_or_typename)().deserialize(raw)


@util.memoize
def scalar(typename):
    """
    Returns scalar type from ROS message data type, like "uint8" from "uint8[100]".

    Returns type unchanged if already a scalar.
    """
    return typename[:typename.index("[")] if "[" in typename else typename


def to_nsec(val):
    """Returns value in nanoseconds if value is ROS1 time/duration, else value."""
    return val.to_nsec() if isinstance(val, genpy.TVal) else val


def to_sec(val):
    """Returns value in seconds if value is ROS1 time/duration, else value."""
    return val.to_sec() if isinstance(val, genpy.TVal) else val


def to_sec_nsec(val):
    """Returns value as (seconds, nanoseconds) if value is ROS1 time/duration, else value."""
    return (val.secs, val.nsecs) if isinstance(val, genpy.TVal) else val


def to_time(val):
    """Returns value as ROS1 time if convertible (int/float/duration/datetime/decimal), else value."""
    result = val
    if isinstance(val, decimal.Decimal):
        result = rospy.Time(int(val), float(val % 1) * 10**9)
    elif isinstance(val, datetime.datetime):
        result = rospy.Time(int(val.timestamp()), 1000 * val.microsecond)
    elif isinstance(val, (float, int)):
        result = rospy.Time(val)
    elif isinstance(val, rospy.Duration):
        result = rospy.Time(val.secs, val.nsecs)
    return result


__all__ = [
    "AnyMsg", "Bag", "ROSLogHandler", "FAMILY", "PARAM_SEPARATOR", "PRIVATE_PREFIX",
    "PY_LOG_LEVEL_TO_ROSPY_LEVEL", "ROS_ALIAS_TYPES", "ROS_TIME_CLASSES", "ROS_TIME_TYPES",
    "canonical", "create_client", "create_publisher", "create_rate", "create_service",
    "create_subscriber", "create_timer", "delete_param", "deserialize_message",
    "destroy_entity", "format_param_name", "get_logger", "get_message_class",
    "get_message_definition", "get_message_fields", "get_message_type",
    "get_message_type_hash", "get_message_value", "get_namespace", "get_node_name",
    "get_nodes", "get_param", "get_param_names", "get_params", "get_rostime",
    "get_service_definition", "get_service_request_class", "get_service_response_class",
    "get_services", "get_topics", "has_param", "init_node", "init_params", "is_ros_message",
    "is_ros_service", "is_ros_time", "make_duration", "make_time", "ok", "register_init",
    "remap_name", "resolve_name", "scalar", "serialize_message", "set_param", "shutdown",
    "spin", "spin_once", "spin_until_future_complete", "start_spin", "to_nsec", "to_sec",
    "to_sec_nsec", "to_time"
]
