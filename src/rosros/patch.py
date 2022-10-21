"""
Patches ROS1 classes with ROS2-compatible members, and vice versa.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     23.02.2022
@modified    21.10.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.patch
import functools
import numbers
import os
import threading
import time
import traceback


rospy = rclpy = ros = None
if os.getenv("ROS_VERSION") != "2":
    import genpy
    import rospy

    from . rclify.clock import Clock, ClockType
    from . import ros1 as ros
else:
    import rclpy.duration
    import rclpy.node
    import rclpy.publisher
    import rclpy.service
    import rclpy.subscription
    import rclpy.time
    import rclpy.timer
    from rclpy.clock import ClockType

    from . import ros2 as ros
    from . rospify import AnyMsg

from . import util


## Extra attributes for objects with __slots__, as {id(obj): {attrname: attrval}}
EXTRA_ATTRS = {}

## Whether generic patching has been done during this runtime.
PATCHED = False

## Whether full patching has been done during this runtime.
PATCHED_FULL = False


def patch_ros1():
    """Patches ROS1 classes with generic compatibility with ROS2."""
    global PATCHED
    if PATCHED: return

    def client_call_async(self, *args, **kwargs):
        """
        Make a service request and asyncronously get the result.

        Service is invoked in a background thread.

        This accepts either a request message instance,
        or positional and keyword arguments to create a new request instance.

        @return  `asyncio.Future`-conforming interface that completes when the request does
        """
        reqcls = self.request_class
        attributes = functools.partial(ros.get_message_fields, reqcls)
        req = util.ensure_object(reqcls, attributes, *args, **kwargs)
        return util.start_future(self.call, req)

    rospy.ServiceProxy.call_async       = client_call_async
    rospy.ServiceProxy.wait_for_service = ServiceProxy__wait_for_service
    rospy.ServiceProxy.service_is_ready = ServiceProxy__service_is_ready

    rospy.Subscriber.__init__                     = Subscriber__init
    rospy.topics._SubscriberImpl.receive_callback = SubscriberImpl__receive_callback

    PATCHED = True


def patch_ros1_rclify():
    """Patches ROS1 classes with full stand-ins for ROS2 methods and properties."""
    global PATCHED_FULL
    patch_ros1()
    if PATCHED_FULL: return

    rospy.ServiceProxy.destroy                = rospy.ServiceProxy.close
    rospy.ServiceProxy.remove_pending_request = ServiceProxy__remove_pending_request

    rospy.Service.destroy = rospy.Service.shutdown

    rospy.Publisher.topic_name             = property(lambda self: self.name)
    rospy.Publisher.assert_liveliness      = Publisher__assert_liveliness
    rospy.Publisher.destroy                = rospy.Publisher.unregister
    rospy.Publisher.get_subscription_count = rospy.Publisher.get_num_connections

    rospy.Subscriber.topic_name = property(lambda self: self.name)
    rospy.Subscriber.destroy    = rospy.Subscriber.unregister

    rospy.Time.__init__            = Time__init
    rospy.Time.__del__             = Time__del
    rospy.Time.nanoseconds         = property(rospy.Duration.to_nsec)
    rospy.Time.clock_type          = property(Time__clock_type)
    rospy.Time.seconds_nanoseconds = Time__seconds_nanoseconds
    rospy.Time.to_msg              = Temporal__to_msg
    rospy.Time.from_msg            = Time__from_msg

    rospy.Duration.__init__    = Duration__init
    rospy.Duration.nanoseconds = property(rospy.Duration.to_nsec)
    rospy.Duration.to_msg      = Temporal__to_msg
    rospy.Duration.from_msg    = Duration__from_msg

    rospy.Rate.destroy               = lambda self: None

    rospy.Timer.__init__             = Timer__init
    rospy.Timer.run                  = Timer__run
    rospy.Timer.cancel               = Timer__cancel
    rospy.Timer.reset                = Timer__reset
    rospy.Timer.shutdown             = Timer__shutdown
    rospy.Timer.destroy              = Timer__shutdown
    rospy.Timer.is_canceled          = Timer__is_canceled
    rospy.Timer.is_ready             = Timer__is_ready
    rospy.Timer.time_since_last_call = Timer__time_since_last_call
    rospy.Timer.time_until_next_call = Timer__time_until_next_call
    rospy.Timer.clock                = property(Timer__clock)
    rospy.Timer.timer_period_ns      = property(Timer__timer_period_ns,
                                                Timer__timer_period_ns__setter)

    PATCHED_FULL = True


def patch_ros2():
    """Patches ROS2 classes with full stand-ins for ROS1 methods and properties."""
    global PATCHED, PATCHED_FULL
    if PATCHED: return

    rclpy.client.Client.call           = client_call_wrapper(rclpy.client.Client.call)
    rclpy.client.Client.call_async     = client_call_wrapper(rclpy.client.Client.call_async)
    rclpy.client.Client.__call__       = rclpy.client.Client.call
    rclpy.client.Client.close          = rclpy.client.Client.destroy
    rclpy.client.Client.service_class  = property(lambda self: self.srv_type)
    rclpy.client.Client.request_class  = property(lambda self: self.srv_type.Request)
    rclpy.client.Client.response_class = property(lambda self: self.srv_type.Response)
    rclpy.client.Client.resolved_name  = property(lambda self: self.srv_name)

    rclpy.publisher.Publisher.publish             = publish_wrapper(rclpy.publisher.Publisher.publish)
    rclpy.publisher.Publisher.get_num_connections = rclpy.publisher.Publisher.get_subscription_count
    rclpy.publisher.Publisher.unregister          = rclpy.publisher.Publisher.destroy
    rclpy.publisher.Publisher.resolved_name       = rclpy.publisher.Publisher.topic_name
    rclpy.publisher.Publisher.name                = property(lambda self: self.topic)
    rclpy.publisher.Publisher.data_class          = property(lambda self: self.msg_type)
    rclpy.publisher.Publisher.type                = property(lambda self: ros.get_message_type(self.msg_type))
    rclpy.publisher.Publisher.md5sum              = property(lambda self: ros.get_message_type_hash(self.msg_type))

    rclpy.service.Service.__init__ = Service__init
    rclpy.service.Service.spin     = Service__spin
    rclpy.service.Service.shutdown = rclpy.service.Service.destroy
    rclpy.service.Service.service_class  = property(lambda self: self.srv_type)
    rclpy.service.Service.request_class  = property(lambda self: self.srv_type.Request)
    rclpy.service.Service.response_class = property(lambda self: self.srv_type.Response)
    rclpy.service.Service.resolved_name  = property(lambda self: self.srv_name)

    rclpy.subscription.Subscription.__init__            = Subscription__init
    rclpy.subscription.Subscription.get_num_connections = lambda self: 0  # Data not available in ROS2
    rclpy.subscription.Subscription.unregister          = rclpy.subscription.Subscription.destroy
    rclpy.subscription.Subscription.resolved_name       = rclpy.subscription.Subscription.topic_name
    rclpy.subscription.Subscription.name                = property(lambda self: self.topic)
    rclpy.subscription.Subscription.data_class          = property(lambda self: self.msg_type)
    rclpy.subscription.Subscription.type                = property(lambda self: ros.get_message_type(self.msg_type))
    rclpy.subscription.Subscription.md5sum              = property(lambda self: ros.get_message_type_hash(self.msg_type))
    rclpy.subscription.Subscription.callback_args       = property(Subscription__callback_args)
    rclpy.subscription.Subscription._invoke_callbacks   = Subscription__invoke_callbacks

    rclpy.duration.Duration.__init__     = Duration__init
    rclpy.duration.Duration.__abs__      = Duration__abs
    rclpy.duration.Duration.__neg__      = Duration__neg
    rclpy.duration.Duration.__add__      = Duration__add
    rclpy.duration.Duration.__radd__     = Duration__add
    rclpy.duration.Duration.__sub__      = Duration__sub
    rclpy.duration.Duration.__mul__      = Duration__mul
    rclpy.duration.Duration.__rmul__     = Duration__mul
    rclpy.duration.Duration.__floordiv__ = Duration__floordiv
    rclpy.duration.Duration.__truediv__  = Duration__truediv
    rclpy.duration.Duration.__mod__      = Duration__mod
    rclpy.duration.Duration.__divmod__   = Duration__divmod
    rclpy.duration.Duration.__bool__     = TVal__is_nonzero
    rclpy.duration.Duration.is_zero      = TVal__is_zero
    rclpy.duration.Duration.to_sec       = TVal__to_sec
    rclpy.duration.Duration.to_nsec      = TVal__to_nsec
    rclpy.duration.Duration.secs         = property(TVal__secs)
    rclpy.duration.Duration.nsecs        = property(TVal__nsecs)

    rclpy.time.Time.__init__ = Time__init
    rclpy.time.Time.__bool__ = TVal__is_nonzero
    rclpy.time.Time.is_zero  = TVal__is_zero
    rclpy.time.Time.to_sec   = TVal__to_sec
    rclpy.time.Time.to_nsec  = TVal__to_nsec
    rclpy.time.Time.secs     = property(TVal__secs)
    rclpy.time.Time.nsecs    = property(TVal__nsecs)

    rclpy.timer.Rate.remaining = Rate__remaining

    rclpy.timer.Timer.shutdown  = rclpy.timer.Timer.destroy
    rclpy.timer.Timer.ident     = property(Timer__ident)
    rclpy.timer.Timer.native_id = property(Timer__ident)
    rclpy.timer.Timer.getName   = Timer__getName
    rclpy.timer.Timer.setName   = Timer__setName
    rclpy.timer.Timer.name      = property(Timer__getName, Timer__setName)
    rclpy.timer.Timer.isDaemon  = Timer__isDaemon
    rclpy.timer.Timer.setDaemon = Timer__setDaemon
    rclpy.timer.Timer.daemon    = property(Timer__isDaemon, Timer__setDaemon)
    rclpy.timer.Timer.join      = Timer__join
    rclpy.timer.Timer.is_alive  = Timer__is_alive
    rclpy.timer.Timer.isAlive   = Timer__is_alive


    PATCHED = PATCHED_FULL = True


def set_extra_attribute(obj, name, value):
    """Sets value for object patched attribute."""
    if hasattr(obj, name):
        setattr(obj, name, value)
    else:
        EXTRA_ATTRS.setdefault(id(obj), {})[name] = value


if rospy:  # Patch-functions to apply on ROS1 classes, to achieve parity with ROS2 API

    def Publisher__assert_liveliness(self):
        """Does nothing (ROS2 compatibility stand-in)."""


    def ServiceProxy__remove_pending_request(self, future):
        """
        Does nothing (ROS2 compatibility stand-in).

        @param   future  ignored (ROS2 compatibility stand-in)
        """

    def ServiceProxy__service_is_ready(self):
        """
        Returns whether service is ready.

        Provides rospy.ServiceProxy with service_is_ready() like rclpy.client.Client has.

        @return  True if a server is ready, False otherwise
        """
        # Heuristically chosen 1ms; rospy might not check service at all if timeout too short
        return self.wait_for_service(1E-3)

    def ServiceProxy__wait_for_service(self, timeout=None, timeout_sec=None):
        """
        Waits for a service server to become ready.

        Returns as soon as a server becomes ready or if the timeout expires.

        Gives rospy.ServiceProxy.wait_for_service the behavior of
        rclpy.client.Client.wait_for_service.

        @param   timeout      time to wait, as seconds or ROS Duration. If None, then wait forever.
        @param   timeout_sec  seconds to wait; if None, wait forever (ROS2 compatibility stand-in)
        @return               True if server became ready or False on a timeout
        """
        if timeout_sec is not None: timeout = timeout_sec
        try:
            rospy.wait_for_service(self.resolved_name, ros.to_nsec(timeout))
        except rospy.ROSInterruptException: raise
        except rospy.ROSException:
            return False
        return True

    ROS1_Subscriber__init                 = rospy.Subscriber.__init__
    ROS1_SubscriberImpl__receive_callback = rospy.topics._SubscriberImpl.receive_callback

    def Subscriber__init(self, name, data_class, callback=None, callback_args=None,
                         queue_size=None, buff_size=65536, tcp_nodelay=False):
        """Wraps rospy.Subscriber.__init__() to add raw bytes autoconversion support."""
        ROS1_Subscriber__init(self, name, data_class, callback, callback_args,
                              queue_size, buff_size, tcp_nodelay)
        self._anymsg = issubclass(data_class, rospy.AnyMsg)  # Whether to return AnyMsg or bytes
        self.impl._interface = self

    def SubscriberImpl__receive_callback(self, msgs, connection):
        """Wraps rospy.topics.SubscriberImpl.receive_callback() with raw bytes autoconversion."""
        if not self._interface._anymsg and issubclass(self.data_class, rospy.AnyMsg):
            msgs = [x._buff for x in msgs]
        ROS1_SubscriberImpl__receive_callback(self, msgs, connection)


    ROS1_Duration__init = rospy.Duration.__init__

    def Duration__init(self, secs=0, nsecs=0, *, seconds=0, nanoseconds=0):
        """Constructs a new `Duration`, using either ROS1 or ROS2 arguments."""
        if secs or nsecs: seconds, nanoseconds = secs, nsecs
        ROS1_Duration__init(self, seconds, nanoseconds)

    def Temporal__to_msg(self):
        """Returns a new Time or Duration from this."""
        return type(self)(self.secs, self.nsecs)

    @classmethod
    def Duration__from_msg(cls, msg):
        """Returns a new Duration from given."""
        return cls(msg.secs, msg.nsecs)


    ROS1_Time__init = rospy.Time.__init__

    def Time__init(self, secs=0, nsecs=0, *, seconds=0, nanoseconds=0,
                   clock_type=ClockType.SYSTEM_TIME):
        """Constructs a new ROS1 `Time`, using either ROS1 or ROS2 arguments."""
        if secs or nsecs: seconds, nanoseconds = secs, nsecs
        ROS1_Time__init(self, seconds, nanoseconds)
        # Keep extra attributes externally, as Time attributes are set in __slots__
        if clock_type != ClockType.SYSTEM_TIME:
            EXTRA_ATTRS.setdefault(id(self), {})["_clock_type"] = clock_type

    def Time__del(self):
        """Clear object-related data, if any."""
        EXTRA_ATTRS.pop(id(self), None)

    def Time__seconds_nanoseconds(self):
        """Returns time as (seconds, nanoseconds)."""
        return (self.secs, self.nsecs)

    def Time__clock_type(self):
        """Returns the ClockType of this Time."""
        attrs = EXTRA_ATTRS.get(id(self)) or {}
        return attrs.get("_clock_type", ClockType.SYSTEM_TIME)

    @classmethod
    def Time__from_msg(cls, msg, clock_type=ClockType.ROS_TIME):
        """Returns a new Time from given."""
        return cls(msg.secs, msg.nsecs, clock_type=clock_type)


    ROS1_Timer__init = rospy.Timer.__init__

    def Timer__init(self, period, callback, oneshot=False, reset=False):
        """
        Constructs a new ROS1 Timer.

        Plugs in ROS2 API compatibility: cancelling, resetting, and public timing info.

        @param   period    desired period between callbacks, as rospy.Duration
        @param   callback  callback to be called, taking rospy.TimerEvent
        @param   oneshot   if True, fire only once, otherwise fire continuously
                           until shutdown is called [default: False]
        @param   reset     if True, timer is reset when rostime moved backward. [default: False]
        """
        self._paused    = False
        self._wakeable  = threading.Event()  # Interruptable sleeper
        self._time_last = None               # rospy.Time() of start or last call or reset
        self._time_next = None               # rospy.Time() of next expected call
        self._clock     = Clock()
        ROS1_Timer__init(self, period, callback, oneshot, reset)  # Starts thread

    def Timer__shutdown(self):
        """Stop firing callbacks."""
        self._shutdown = True
        self._wakeable.set()

    def Timer__run(self):
        """Override rospy.Timer.run() to support canceling and resetting, and call timestamps."""
        self._time_last = self._clock.now()
        self._time_next = self._time_last + self._period
        last_expected, last_real, last_duration = None, None, None
        self._wakeable.clear()
        while not rospy.core.is_shutdown() and not self._shutdown:
            try:
                rostimer_sleep_on_event(self._period.to_sec(), event=self._wakeable)
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                if not self._reset:
                    raise
            except rospy.exceptions.ROSInterruptException:
                if rospy.core.is_shutdown():
                    break
                raise
            if self._shutdown:
                break
            if self._paused:
                self._wakeable.wait()
            if self._shutdown:
                break
            if self._paused is None:
                self._paused = False
                self._time_last = self._clock.now()
                self._time_next = self._time_last + self._period
                last_expected, last_real, last_duration = None, None, None
                self._wakeable.clear()
                continue
            current_real = self._clock.now()
            start = time.time()
            self._time_last = self._clock.now()
            self._callback(rospy.timer.TimerEvent(last_expected, last_real, self._time_next,
                                                  current_real, last_duration))
            if self._oneshot:
                break
            last_duration = time.time() - start
            last_expected, last_real = self._time_next, current_real
            self._time_next += self._period

    def Timer__cancel(self):
        """Sets timer on pause, until reset or shutdown."""
        if self._paused: return

        self._paused = True
        self._wakeable.set()

    def Timer__reset(self):
        """Sets last call time to now, postponing next callback by full period; uncancels if canceled."""
        self._paused = None
        self._wakeable.set()

    def Timer__clock(self):
        """Returns the Clock used by this timer."""
        return self._clock

    def Timer__timer_period_ns(self):
        """Returns timer period in nanoseconds."""
        return self._period.to_nsec()

    def Timer__timer_period_ns__setter(self, value):
        """Sets timer period in nanoseconds, resets timer if running."""
        self._period = rospy.Duration(nsecs=value)
        if not self._paused: self._wakeable.set()

    def Timer__is_canceled(self):
        """Returns whether the timer has been canceled (paused until reset or shutdown)."""
        return bool(self._paused)

    def Timer__is_ready(self):
        """Returns whether timer callback should be called right now."""
        if self._paused or self._time_last is None:
            return False
        return self._period >= (self._time_next - self._time_last)

    def Timer__time_since_last_call(self):
        """Returns nanoseconds since last timer callback (ROS2 compatibility stand-in)."""
        return None if self._time_last is None else (self._clock.now() - self._time_last).to_nsec()

    def Timer__time_until_next_call(self):
        """Returns nanoseconds until next timer callback (ROS2 compatibility stand-in)."""
        if self._shutdown or self._paused or self._time_next is None:
            return None
        return (self._time_next - self._clock.now()).to_nsec()



    def rostimer_sleep_on_event(duration, event=None):
        """
        Sleeps for the specified duration in ROS time, or until event gets set.

        Returns immediately if duration is negative or event is set.

        @param   duration  seconds (or rospy.Duration) to sleep
        @param   event     threading.Event to wait() on, if any,
                           will return early if event gets set

        @throws  ROSInterruptException           if ROS shutdown occurs before sleep completes
        @throws  ROSTimeMovedBackwardsException  if ROS time is set backwards
        """
        if event and event.is_set(): return

        if rospy.rostime.is_wallclock():
            if isinstance(duration, genpy.Duration):
                duration = duration.to_sec()
            if duration < 0:
                return
            event.wait(duration) if event else rospy.rostime.wallsleep(duration)
        else:
            initial_rostime = rospy.rostime.get_rostime()
            if not isinstance(duration, genpy.Duration):
                duration = genpy.Duration.from_sec(duration)

            rostime_cond = rospy.rostime.get_rostime_cond()

            if initial_rostime == genpy.Time(0):
                # break loop if time is initialized or node is shutdown
                while initial_rostime == genpy.Time(0) and \
                          not rospy.core.is_shutdown():
                    with rostime_cond:
                        rostime_cond.wait(0.3)
                    initial_rostime = rospy.rostime.get_rostime()
                    if event and event.is_set(): return

            sleep_t = initial_rostime + duration

            # break loop if sleep_t is reached, time moves backwards, or node is shutdown
            while rospy.rostime.get_rostime() < sleep_t \
            and rospy.rostime.get_rostime() >= initial_rostime and not rospy.core.is_shutdown():
                with rostime_cond:
                    rostime_cond.wait(0.5)
                if event and event.is_set(): return

            if rospy.rostime.get_rostime() < initial_rostime:
                time_jump = (initial_rostime - rospy.rostime.get_rostime()).to_sec()
                raise rospy.exceptions.ROSTimeMovedBackwardsException(time_jump)
            if rospy.core.is_shutdown():
                raise rospy.exceptions.ROSInterruptException("ROS shutdown request")



elif rclpy:  # Patch-functions to apply on ROS2 classes, to achieve parity with ROS1 API

    def client_call_wrapper(call):
        """Returns service client call-function wrapped to ensure invoke with request instance."""
        def inner(self, *args, **kwargs):
            reqcls = self.srv_type.Request
            attributes = functools.partial(ros.get_message_fields, reqcls)
            req = util.ensure_object(reqcls, attributes, *args, **kwargs)
            return call(self, req)
        return functools.update_wrapper(inner, call)

    def service_serve_wrapper(self, serve):
        """Returns service serve-function wrapped to ensure return with response instance."""
        def inner(req, resp):
            resp, respcls, args, kwargs = serve(req), self.srv_type.Response, None, None
            if   isinstance(resp, (list, tuple)): args   = resp
            elif isinstance(resp, dict):          kwargs = resp
            if args is not None or kwargs is not None:
                attributes = functools.partial(ros.get_message_fields, respcls)
                resp = util.ensure_object(respcls, attributes, *args or [], **kwargs or {})
            return resp
        return functools.update_wrapper(inner, serve)

    def publish_wrapper(publish):
        """Returns publish-function wrapped to ensure invoke with message instance."""
        def inner(self, *args, **kwargs):
            attributes = functools.partial(ros.get_message_fields, self.msg_type)
            msg = util.ensure_object(self.msg_type, attributes, *args, **kwargs)
            return publish(self, msg)
        return functools.update_wrapper(inner, publish)



    ROS2_Duration__init = rclpy.duration.Duration.__init__

    def Duration__init(self, secs=0, nsecs=0, *, seconds=0, nanoseconds=0):
        """Constructs a new ROS2 `Duration`, using either ROS1 or ROS2 arguments."""
        if secs or nsecs: seconds, nanoseconds = secs, nsecs
        ROS2_Duration__init(self, seconds=seconds, nanoseconds=nanoseconds)

    def Duration__neg(self):
        """Returns negative value of this duration."""
        return self.__class__(nanoseconds=-self.nanoseconds)

    def Duration__abs(self):
        """Returns absolute value of this duration."""
        if self.nanoseconds >= 0:
            return self
        return self.__neg__()

    def Duration__add(self, other):
        """
        Returns a new duration, adding given duration to this.

        @param   other  another `Duration` instance
        @returns        new `Duration` instance
        """
        if isinstance(other, rclpy.duration.Duration):
            return self.__class__(nanoseconds=self.nanoseconds + other.nanoseconds)
        return NotImplemented

    def Duration__sub(self, other):
        """
        Subtracts a duration from this duration.

        @param   other  another `Duration` instance
        @returns        new `Duration` instance
        """
        if isinstance(other, rclpy.duration.Duration):
            return self.__class__(nanoseconds=self.nanoseconds - other.nanoseconds)
        return NotImplemented

    def Duration__mul(self, val):
        """
        Multiplies this duration by an integer or float.

        @param   val   multiplication factor, as `int` or `float`
        @return        new `Duration` instance
        """
        if isinstance(val, numbers.Number):
            return self.__class__(nanoseconds=self.nanoseconds * val)
        return NotImplemented

    def Duration__floordiv(self, val):
        """
        Floor-divides this duration by a number or a duration.

        @param   val  division factor, as `int` or `float` or `Duration` to divide by
        @return       new `Duration` instance if divided by a number,
                      or seconds as integer if divided by a `Duration`
        """
        if isinstance(val, numbers.Number):
            return self.__class__(seconds=self.to_sec() // val)
        if isinstance(val, rclpy.duration.Duration):
            return int(self.to_sec() // val.nanoseconds)
        return NotImplemented

    def Duration__truediv(self, val):
        """
        Divides this duration by an integer or float.

        @param   val  division factor, as `int` or `float` or `Duration` to divide by
        @return       new `Duration` instance if divided by a number,
                      or seconds as float if divided by a `Duration`
        """
        if isinstance(val, numbers.Number):
            return self.__class__(seconds=self.to_sec() / val)
        if isinstance(val, rclpy.duration.Duration):
            return self.to_sec() / val.to_sec()
        return NotImplemented

    def Duration__mod(self, val):
        """
        Returns the remainder of dividing this duration by another duration.

        @param   val  modulo factor, as `int` or `float` to mod by
        @return       new `Duration` instance of remaining time after mod
        """
        if isinstance(val, rclpy.duration.Duration):
            return self.__class__(nanoseconds=self.nanoseconds % val.nanoseconds)
        return NotImplemented

    def Duration__divmod(self, val):
        """
        Returns the divmod result for this and another duration.

        @param   val  division factor, as `Duration` to divide by
        @return       (division floored result as `int`, remaining time as `Duration`)
        """
        if isinstance(val, rclpy.duration.Duration):
            quotient, remainder = divmod(self.nanoseconds, val.nanoseconds)
            return int(quotient), self.__class__(nanoseconds=remainder)
        return NotImplemented


    ROS2_Service__init = rclpy.service.Service.__init__

    def Service__init(self, service_handle, srv_type, srv_name, callback,
                      callback_group, qos_profile):
        """Wraps Service.__init__() to support returning list or dict from server callback."""
        callback = service_serve_wrapper(self, callback)
        ROS2_Service__init(self, service_handle, srv_type, srv_name, callback,
                           callback_group, qos_profile)

    def Service__spin(self):
        """Spins ROS2 until service or node shutdown."""
        try:
            while not ros.SHUTDOWN and self.handle:
                ros.spin_once(0.5)
        except KeyboardInterrupt:
            ros.get_logger().debug("keyboard interrupt, shutting down")


    ROS2_Subscription__init = rclpy.subscription.Subscription.__init__

    def Subscription__init(self, subscription_handle, msg_type, topic, callback,
                           callback_group, qos_profile, raw, event_callbacks):
        """Wraps Subscription.__init__() to support AnyMsg and multiple callbacks."""
        ROS2_Subscription__init(self, subscription_handle, msg_type, topic, self._invoke_callbacks,
                                callback_group, qos_profile, raw, event_callbacks)
        self.impl = type("", (), {})()  # Dummy object
        self.impl.add_callback    = functools.partial(Subscription__add_callback,    self)
        self.impl.remove_callback = functools.partial(Subscription__remove_callback, self)
        self._callbacks = getattr(self, "_callbacks", [(callback, None)])
        self._anymsg    = False

    def Subscription__callback_args(self):
        """Returns callback args given in constructor."""
        return self._callbacks[0][-1] if self._callbacks else None

    def Subscription__invoke_callbacks(self, msg):
        """Handler for received message, invokes all added callbacks."""
        if self._anymsg and isinstance(msg, bytes):
            msg = AnyMsg().deserialize(msg)
        for cb, arg in list(self._callbacks):
            try:
                cb(msg) if arg is None else cb(msg, arg)
            except Exception:
                t = ("during shutdown, " if ros.SHUTDOWN else "") + "bad callback: %s\n%s"
                ros.get_logger().warning(t, cb, traceback.format_exc())

    def Subscription__add_callback(self, cb, cb_args):
        """
        Registers a callback to be invoked whenever a new message is received

        @param   cb        callback function to invoke with message data instance, i.e. fn(data).
                           If callback args is set, they will be passed in as the second argument.
        @param   cb_cargs  additional arguments to pass to callback
        """
        self._callbacks.append((cb, cb_args))

    def Subscription__remove_callback(self, cb, cb_args):
        """
        Unregisters a message callback.

        @param   cb        callback function
        @param   cb_cargs  additional arguments associated with callback

        @throws  KeyError  if no matching callback
        """
        matches = [x for x in self._callbacks if x[0] == cb and x[1] == cb_args]
        if matches:
            new_callbacks = self._callbacks[:]
            # remove the first match
            new_callbacks.remove(matches[0])
            self._callbacks = new_callbacks
        if not matches:
            raise KeyError("no matching cb")


    def Rate__remaining(self):
        """Returns time remaining for rate to sleep, as `Duration`."""
        return rclpy.duration.Duration(nanoseconds=self._timer.time_until_next_call())


    ROS2_Timer__init = rclpy.timer.Timer.__init__

    TIMER_COUNTER = 0

    def Timer__init(self, callback, callback_group, timer_period_ns, clock, *, context=None):
        """Constructs a new ROS2 `Timer`."""
        global TIMER_COUNTER
        ROS2_Timer__init(self, callback, callback_group, timer_period_ns, clock, context=context)
        TIMER_COUNTER += 1
        self._name = "%s-%s" % (self.handle.name, TIMER_COUNTER)

    def Timer__ident(self):
        """Returns timer identifier as a non-negative integer."""
        return id(self)

    def Timer__getName(self):
        """Returns timer name."""
        return self._name

    def Timer__setName(self, name):
        """Sets timer name."""
        self._name = name

    def Timer__isDaemon(self):
        """Returns `True`."""
        return True

    def Timer__setDaemon(self, daemonic):
        """Raises error."""
        raise RuntimeError("cannot set daemon status of running thread")

    def Timer__join(self):
        """Returns when timer has terminated."""
        while self.handle and self.is_ready():
            time.sleep(1)

    def Timer__is_alive(self):
        """Returns whether timer is still running."""
        return self.handle and self.is_ready()


    ROS2_Time__init = rclpy.time.Time.__init__

    def Time__init(self, secs=0, nsecs=0, *, seconds=0, nanoseconds=0,
                   clock_type=ClockType.SYSTEM_TIME):
        """Constructs a new ROS2 `Time`, using either ROS1 or ROS2 arguments."""
        if secs or nsecs: seconds, nanoseconds = secs, nsecs
        ROS2_Time__init(self, seconds=seconds, nanoseconds=nanoseconds, clock_type=clock_type)

    def TVal__is_nonzero(self):
        """Returns whether value is not zero."""
        return self.nanoseconds != 0

    def TVal__is_zero(self):
        """Returns whether value is zero."""
        return self.nanoseconds == 0

    def TVal__to_sec(self):
        """Returns value in seconds as `float`."""
        return ros.to_sec(self)

    def TVal__to_nsec(self):
        """Returns value in nanoseconds as `int`."""
        return ros.to_nsec(self)

    def TVal__secs(self):
        """Returns the seconds portion as `int`."""
        return self.nanoseconds // 10**9

    def TVal__nsecs(self):
        """Returns the nanoseconds portion as `int`."""
        return self.nanoseconds % 10**9



__all__ = [
    "patch_ros1", "patch_ros1_rclify", "patch_ros2", "set_extra_attribute"
]
