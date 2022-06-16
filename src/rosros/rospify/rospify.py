"""
Stand-in for `rospy` in ROS2.

Covers the interface of ROS1 `rospy`.

------------------------------------------------------------------------------
This file is part of rosros - simple unified interface to ROS1 / ROS2.
Released under the BSD License.

@author      Erki Suurjaak
@created     20.02.2022
@modified    30.05.2022
------------------------------------------------------------------------------
"""
## @namespace rosros.rospify.rospify
from . core       import is_shutdown, signal_shutdown, get_node_uri, get_ros_root, \
                         parse_rosrpc_uri, \
                         logdebug, logwarn, loginfo, logout, logerr, logfatal, \
                         logdebug_throttle, logwarn_throttle, loginfo_throttle, logerr_throttle, \
                         logfatal_throttle, \
                         logdebug_throttle_identical, logwarn_throttle_identical, \
                         loginfo_throttle_identical, logerr_throttle_identical, \
                         logfatal_throttle_identical, \
                         logdebug_once, logwarn_once, loginfo_once, logerr_once, logfatal_once
from . exceptions import ParameterInvalid, ROSException, ROSInitException, ROSInternalException, \
                         ROSInterruptException, ROSSerializationException, \
                         ROSTimeMovedBackwardsException, TransportException, \
                         TransportInitError, TransportTerminated
from . client     import delete_param, get_master, get_param, get_param_cached, get_param_names, \
                         get_published_topics, has_param, init_node, myargv, on_shutdown, \
                         search_param, set_param, spin, wait_for_message, \
                         DEBUG, INFO, WARN, ERROR, FATAL
from . msg        import AnyMsg, Header
from . msproxy    import MasterProxy
from . names      import get_caller_id, get_name, get_namespace, remap_name, resolve_name
from . rostime    import Duration, Time, get_rostime, get_time
from . service    import Service, ServiceProxy, ServiceException, wait_for_service
from . topics     import Message, SubscribeListener, Publisher, Subscriber
from . timer      import sleep, Rate, Timer, TimerEvent
