rosros
======

Simple unified interface to ROS1 / ROS2 Python API.

Auto-detects ROS version from run-time environment, and provides
equivalent functionality in either ROS version.

Main use-cases:
- helper library for working with ROS messages and data types
- replacement for ROS1 / ROS2 Python API
- drop-in replacement for `rospy` to convert a ROS1 package to a ROS1/ROS2 package
- drop-in replacement for `rclpy` to convert a ROS2 package to a ROS1/ROS2 package


Requires the corresponding ROS Python libraries: `rospy` family for ROS1,
`rclpy` for ROS2.

- [Example usage](#example-usage)
  - [Parameters](#parameters)
  - [Constructor args in publishers and services](#constructor-args-in-publishers-and-services)
  - [Timers](#timers)
  - [Bags](#bags)
- [ROS core functionality](#ros-core-functionality)
  - [Patches in ROS1](#patches-in-ros1)
  - [Patches in ROS2](#patches-in-ros2)
- [API helpers](#api-helpers)
- [Converting an existing package](#converting-an-existing-package)
  - [Existing ROS1 package to ROS1/ROS2 package](#existing-ros1-package-to-ros1ros2-package)
  - [Existing ROS2 package to ROS1/ROS2 package](#existing-ros2-package-to-ros1ros2-package)
- [Running tests](#running-tests)
- [Dependencies](#dependencies)
- [Attribution](#attribution)
- [License](#license)


Example usage
-------------

A simple node that publishes a message on receiving a service request:

```python
import rosros

def on_trigger(req):
    pub.publish(True)
    return {"success": True, "message": "Triggered!"}

rosros.init_node("mynode")
params = rosros.init_params(service="/trigger", topic="/triggerings")
srv = rosros.create_service(params["service"], "std_srvs/Trigger", on_trigger)
pub = rosros.create_publisher(params["topic"], "std_msgs/Bool", latch=True, queue_size=2)
rosros.spin()
```

Equivalent ROS1 code using rospy:

```python
import rospy, std_msgs.msg, std_srvs.srv

def on_trigger(req):
    pub.publish(True)
    return {"success": True, "message": "Triggered!"}

rospy.init_node("mynode")
service = rospy.get_param("~service", "/trigger")
topic   = rospy.get_param("~topic",   "/triggerings")
rospy.set_param("~service", service)
rospy.set_param("~topic",   topic)
srv = rospy.Service(service, std_srvs.srv.Trigger, on_trigger)
pub = rospy.Publisher(topic, std_msgs.msg.Bool, latch=True, queue_size=2)
rospy.spin()
```

Equivalent ROS2 code using rclpy:

```python
import rclpy, rclpy.qos, std_msgs.msg, std_srvs.srv

def on_trigger(req, resp):
    pub.publish(std_msgs.msg.Bool(data=True))
    resp.success = True
    resp.message = "Triggered!"
    return resp

rclpy.init()
node = rclpy.create_node("mynode")
service = node.get_parameter_or("service", rclpy.Parameter("", value="/trigger")).value
topic   = node.get_parameter_or("topic",   rclpy.Parameter("", value="/triggerings")).value
if not node.has_parameter("service"):
    node.declare_parameter("service", service)
if not node.has_parameter("topic"):
    node.declare_parameter("topic", topic)
srv = node.create_service(std_srvs.srv.Trigger, service, on_trigger)
qos = rclpy.qos.QoSProfile(depth=2, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
pub = node.create_publisher(std_msgs.msg.Bool, topic, qos)
rclpy.spin(node)
```

Any ROS2-only extras used will be ignored under ROS1, like topic QoS reliability.


### Parameters

rosros provides a convenient way to auto-load all node parameters,
and override or populate additional parameters from a nested dictionary.

```python
rosros.init_node("mynode")
# If the given defaults were not among existing node parameters,
# they will be auto-declared on the node.
params = rosros.init_params({"my": {"nested": {"param1": 5, "param2": True}}})
```


### Constructor args in publishers and services

rosros provides short-hand convenience wrappers for publishing messages,
invoking services and returning service responses, without needing to
create the message or service request/response class instance explicitly.

ROS1 already provides this short-hand in `rospy`,
but ROS2 API requires the instances to be created explicitly.

In rosros:

```python
pub = rosros.create_publisher("/topic", "std_msgs/Bool")
pub.publish(True)
pub.publish(data=True)
pub.publish()

# Service callback need not accept any parameters if request object is irrelevant
srv = rosros.create_service("/service", "std_srvs/SetBool", lambda: {"message": "Triggered!"})

cli = rosros.create_client("/service", "std_srvs/SetBool")
cli.call(True)
cli.call(data=True)
cli(True)
```

Equivalent ROS1 code using rospy:

```python
pub = rospy.Publisher("/topic", std_msgs.msg.Bool, queue_size=0)
pub.publish(True)
pub.publish(data=True)
pub.publish()

srv = rospy.Service("/service", std_srvs.srv.SetBool, lambda req: {"message": "Triggered!"})

cli = rospy.ServiceProxy("/service", std_srvs.srv.SetBool)
cli.call(True)
cli.call(data=True)
cli(True)
```

Equivalent ROS2 code using rospy:

```python
pub = mynode.create_publisher(std_msgs.msg.Bool, "/topic", 0)
pub.publish(std_msgs.msg.Bool(data=True))
pub.publish(std_msgs.msg.Bool(data=True))
pub.publish(std_msgs.msg.Bool())

def callback(req, resp):
  resp.message = "Triggered!"
  return resp
srv = mynode.create_service(std_srvs.srv.SetBool, "/service", callback)

cli = mynode.create_client(std_srvs.srv.SetBool, "/service")
cli.call(std_srvs.srv.SetBool.Request(data=True))
```

### Timers

rosros provides convenience one-shot timers in ROS2 like ROS1 does.

Additionally, rosros provides an option to start invoking the timer callback
immediately, instead of waiting one period before first call.

```python
timer = rosros.create_timer(2, lambda: print("just once, after 2 sec"), oneshot=True)

timer = rosros.create_timer(5, lambda: print("every 5 sec, starting now"), immediate=True)

# Can take seconds, can take an explicit Duration object
timer = rosros.create_timer(rosros.api.make_duration(5), lambda: print("every 5 sec"))
```

### Bags

rosros provides a unified interface for reading and writing ROS bags.

Bag format is the custom ROS format in ROS1, and the SQLite database format in ROS2.

```python
bag = rosros.Bag("my.bag", mode="a")
print(bag)  # Prints bag metainfo, equivalent to "rosbag info"

bag.write("/my/topic", std_msgs.msg.Bool())

for topic, message, stamp in bag:
    print("[%s] %s:  %s" % (rosros.api.to_sec(stamp), topic, message))
```




ROS core functionality
----------------------

Functionality for creating and operating a ROS node.

Objects returned are standard `rospy` / `rclpy` API objects, patched with
additional members for a unified interface _**conforming to ROS1 API**_.


| Name                                | Description                                                                                | Arguments
| ----------------------------------- | ------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------
|                                     | **Startup, spin and shutdown**                                                             | |
| `rosros.init_node`                  | initializes ROS and creates ROS node                                                       | `name, args=None, namespace=None, anonymous=True, log_level=None, enable_rosout=True, multithreaded=True, reentrant=False`
| `rosros.start_spin`                 | sets ROS node spinning forever in a background thread                                      | |
| `rosros.spin`                       | spins ROS node forever                                                                     | |
| `rosros.spin_once`                  | waits until timeout in ROS1; executes one ROS operation or waits until timeout in ROS2     | `timeout=None`
| `rosros.spin_until_future_complete` | spins ROS until future complete or timeout reached or ROS shut down                        | `future, timeout=None`
| `rosros.ok`                         | returns whether ROS has been initialized and is not shut down                              | |
| `rosros.shutdown`                   | shuts down live ROS node, if any                                                           | |
|                                     |                                                                                            | |
|                                     | **Parameters**                                                                             | |
| `rosros.init_params`                | sets all parameters on node from defaults dictionary, returns full parameters dictionary.  | `defaults=None, **defaultkws`
|                                     | In ROS2, auto-declares unregistered parameters.                                            | |
| `rosros.has_param`                  | returns whether the parameter exists                                                       | `name`
| `rosros.get_param`                  | returns parameter value from current ROS node                                              | `name, default=None, autoset=True`
| `rosros.get_param_names`            | returns the names of all current ROS node parameters                                       | |
| `rosros.get_params`                 | returns the current ROS node parameters, by default as nested dictionary                   | `nested=True`
| `rosros.set_param`                  | sets a parameter on the node.                                                              | `name, value, descriptor=None`
|                                     | In ROS2, parameter will be auto-declared if unknown so far.                                | |
| `rosros.delete_param`               | deletes parameter from the node                                                            | `name`
|                                     |                                                                                            | |
|                                     | **Topics, services, timers and rates**                                                     | |
| `rosros.create_publisher`           | returns a ROS publisher instance                                                           | `topic, cls_or_typename, latch=False, queue_size=0, **qosargs`
| `rosros.create_subscriber`          | returns a ROS subscriber instance                                                          | `topic, cls_or_typename, callback, callback_args=None, queue_size=0, raw=False, **qosargs`
| `rosros.create_service`             | returns a ROS service server instance, for providing a service                             | `service, cls_or_typename, callback, **qosargs`
| `rosros.create_client`              | returns a ROS service client instance, for invoking a service                              | `service, cls_or_typename, **qosargs`
| `rosros.create_timer`               | returns a ROS timer instance                                                               | `period, callback, oneshot=False, immediate=False`
| `rosros.create_rate`                | returns a ROS rate instance, for sleeping at a fixed rate                                  | `frequency`
| `rosros.destroy_entity`             | closes the given publisher, subscriber, service client, service server, or timer instance  | `item`
| `rosros.AnyMsg`                     | `rospy.AnyMsg` in ROS1, stand-in class with equivalent functionality in ROS2               | |
|                                     |                                                                                            | |
|                                     | **ROS bags**                                                                               | |
| `rosros.Bag`                        | ROS bag reader and writer;                                                                 | |
|                                     | `rosbag.Bag` in ROS1 with some additional functionality;                                   | |
|                                     | equivalent class in ROS2, using the ROS2 default SQLite format                             | |
|                                     |                                                                                            | |
|                                     | **Information queries**                                                                    | |
| `rosros.get_namespace`              | returns ROS node namespace                                                                 | |
| `rosros.get_node_name`              | returns ROS node full name with namespace                                                  | |
| `rosros.get_nodes`                  | returns all ROS nodes, as `[node full name, ]`                                             | |
| `rosros.get_topics`                 | returns all available ROS topics, as `[(topic name, [type name, ]), ]`                     | |
| `rosros.get_services`               | returns all available ROS services, as `[(service name, [type name, ]), ]`                 | |
| `rosros.remap_name`                 | returns the absolute remapped topic/service name if mapping exists                         | `name, namespace=None`
| `rosros.resolve_name`               | returns absolute remapped name, namespaced under current node if relative or private       | `name, namespace=None`
|                                     |                                                                                            | |
|                                     | **Miscellaneous**                                                                          | |
| `rosros.get_logger`                 | returns `logging.Logger` for logging to ROS log handler;                                   | |
|                                     | logger supports additional keywords `__once__`, `__throttle__`, `__throttle_identical__`   | |
| `rosros.get_rostime`                | returns ROS time instance for current ROS clock time                                       | |
| `rosros.register_init`              | informs `rosros` of ROS having been initialized outside of `init_node()`.                  | `node=None`
|                                     | Giving node as argument is mandatory in ROS2.                                              | |


### Patches in ROS1

Additional functionality patched onto `rospy` classes for convenience:

- `rospy.ServiceProxy.call_async(*args, **kwargs):`
  makes service call in a background thread, returns `asyncio.Future`-like response
- `rospy.ServiceProxy.wait_for_service(timeout=None, timeout_sec=None):`
  waits for service to become available, returns `True`, or `False`on timeout
- `rospy.ServiceProxy.service_is_ready():`
   returns whether service is currently available
- `rospy.Subscriber:`
  callbacks are invoked with serialized message bytes if subscription was created with `raw=True`
- `rosbag.Bag.get_message_definition(msg_or_type):`
   returns ROS1 message type definition full text from bag
- `rosbag.Bag.get_message_class(typename, typehash=None):`
   returns ROS1 message class
- `rosbag.Bag.get_message_type_hash(msg_or_type):`
   returns ROS1 message type MD5 hash
- `rosbag.Bag.get_topic_info():`
  returns topic and message type metainfo as `{(topic, typename, typehash): count}`
- `rosbag.Bag.reindex_file(f):`
  reindexes bag file on disk; makes a temporary copy in file directory


### Patches in ROS2

The following classes are patched with full conformity to their equivalents in ROS:

- `rclpy.client.Client`
- `rclpy.publisher.Publisher`
- `rclpy.service.Service`
- `rclpy.subscription.Subscription`
- `rclpy.duration.Duration`
- `rclpy.time.Time`
- `rclpy.timer.Rate`
- `rclpy.timer.Timer`

E.g. durations support `+-*/`arithmetic, and publisher / subscriber provide `.md5sum` for message type hash.


API helpers
-----------

Functionality for working with message types and data and metainfo.
Can be used as stand-alone library functions without initializing rosros core.

| Name                                    | Description                                                                                   | Arguments
| --------------------------------------- | --------------------------------------------------------------------------------------------- | ---------------------------------------
|                                         | **Message types and instances**                                                               | |
| `rosros.api.get_message_class`          | returns ROS message / service class object, like `std_msgs.msg.Bool` for `"std_msgs/Bool"`    | `msg_or_type`
| `rosros.api.get_message_definition`     | returns ROS message or service request/response type definition text,                         | `msg_or_type, full=True`
|                                         | by default including message subtype definitions                                              | |
| `rosros.api.get_message_fields`         | returns `{field name: field type name}` if ROS message or service request/response, else `{}` | `val`
| `rosros.api.get_message_header`         | returns message `Header`-attribute if any, else `None`                                        | `val`
| `rosros.api.get_message_type`           | returns ROS message / service canonical type name, like `"std_msgs/Header"`                   | `msg_or_cls`
|                                         | or `"*"` for `AnyMsg`                                                                         | |
| `rosros.api.get_message_type_hash`      | returns ROS message / service type MD5 hash                                                   | `msg_or_type`
| `rosros.api.get_message_value`          | returns message attribute value, with numeric arrays converted to lists                       | `msg, name`
| `rosros.api.is_ros_message`             | returns whether value is a ROS message or service request/response class or instance          | `val`
|                                         |                                                                                               | |
|                                         | **Message conversion**                                                                        | |
| `rosros.api.deserialize_message`        | returns ROS message or service request/response instantiated from serialized binary           | `raw, cls_or_typename`
| `rosros.api.dict_to_message`            | returns given ROS message populated from Python dictionary                                    | `dct, msg`
| `rosros.api.message_to_dict`            | returns ROS message as nested Python dictionary;                                              | `msg, replace=None`
|                                         | with optional replacements for certain values like `{math.nan: None}`                         | |
| `rosros.api.message_to_str`             | returns ROS message as an evaluatable string, e.g. `"std_msgs.msg.UInt8(data=0)"`             | `msg, indent=None`
| `rosros.api.serialize_message`          | returns ROS message or service request/response as a serialized binary of `bytes()`           | `msg`
|                                         |                                                                                               | |
|                                         | **Service classes and definitions**                                                           | |
| `rosros.api.get_service_definition`     | returns ROS service type definition text                                                      | `srv_or_type`
| `rosros.api.get_service_request_class`  | returns ROS service request class object                                                      | `srv_or_type`
| `rosros.api.get_service_response_class` | returns ROS service response class object                                                     | `srv_or_type`
| `rosros.api.is_ros_service`             | returns whether value is a ROS service class object                                           | `val`
|                                         |                                                                                               | |
|                                         | **Time and duration**                                                                         | |
| `rosros.api.is_ros_time`                | returns whether value is a ROS time/duration                                                  | `val`
| `rosros.api.make_duration`              | returns a ROS duration                                                                        | `secs=0, nsecs=0`
| `rosros.api.make_time`                  | returns a ROS time                                                                            | `secs=0, nsecs=0`
| `rosros.api.time_category`              | returns "time" or "duration" for time/duration type, else value                               | `msg_or_type`
| `rosros.api.time_message`               | returns ROS2 time/duration from `rclpy` as `builtin_interfaces` or vice versa,                | `val, to_message=True, clock_type=None`
|                                         | or value if not convertible                                                                   | |
| `rosros.api.to_datetime`                | returns value as `datetime.datetime` if value is ROS time/duration, else value                | `val`
| `rosros.api.to_decimal`                 | returns value as `decimal.Decimal` if value is ROS time/duration, else value                  | `val`
| `rosros.api.to_nsec`                    | returns value in nanoseconds if value is ROS time/duration, else value                        | `val`
| `rosros.api.to_sec`                     | returns value in seconds if value is ROS time/duration, else value                            | `val`
| `rosros.api.to_sec_nsec`                | returns value in (seconds, nanoseconds) if value is ROS time/duration, else value             | `val`                     
| `rosros.api.to_time`                    | returns value as ROS time if convertible (int/float/duration/datetime/decimal), else value    | `val`                     
|                                         |                                                                                               | |
|                                         | **Class and type names**                                                                      | |
| `rosros.api.canonical`                  | returns `"pkg/Type"` for `"pkg/subdir/Type"`                                                  | `typename`
| `rosros.api.get_alias_type`             | returns ROS built-in type for alias like `"char"`, if any; reverse of `get_type_alias()`.     | `typename`
|                                         | In ROS1, `byte` and `char` are aliases for `int8` and `uint8`; in ROS2 the reverse.           | |
| `rosros.api.get_type_alias`             | returns alias like `"char"` for ROS built-in type, if any; reverse of `get_alias_type()`.     | `typename`
|                                         | In ROS1, `byte` and `char` are aliases for `int8` and `uint8`; in ROS2 the reverse.           | |
| `rosros.api.make_full_typename`         | returns `"pkg/msg/Type"` or `"pkg/srv/Type"` for `"pkg/Type"`                                 | `typename, category="msg"`
| `rosros.api.scalar`                     | returns scalar type from ROS message data type, like `"uint8"` from `uint8`-array;            | `typename`
|                                         | in ROS2, returns unbounded type, e.g. `"string"` from `"string<=10[<=5]"`.                    | |
|                                         |                                                                                               | |
|                                         | **Miscellaneous**                                                                             | |
| `rosros.api.format_param_name`          | returns parameter name with correct separator for ROS version, and leading sigils stripped    | `name`
| `rosros.api.PARAM_SEPARATOR`            | separator char between ROS parameter namespace parts                                          | |
| `rosros.api.ROS_BUILTIN_TYPES`          | list of ROS built-in numeric and string type names, like `["string", "uint8", ..]`            | |
| `rosros.api.ROS_NUMERIC_TYPES`          | list of ROS built-in numeric type names, like `["bool", "int8", ..]`                          | |
| `rosros.api.ROS_STRING_TYPES`           | list of ROS built-in string type names                                                        | |
| `rosros.api.ROS_TIME_CLASSES`           | ROS time/duration types mapped to type names                                                  | |
| `rosros.api.ROS_TIME_TYPES`             | list of ROS built-in time/duration type names                                                 | |


Converting an existing package
------------------------------

### Existing ROS1 package to ROS1/ROS2 package

rosros can be used as a (mostly) drop-in replacement for rospy
to make a ROS1 package equally usable under ROS2 - if the code relies only
on `rospy`, and does not go too deeply into using ROS1 specifics like `rosgraph`.

`rospy` imports can be replaced with `rosros.rospify`, e.g.

```python
import threading
from rosros import rospify as rospy
from std_msgs.msg import String

rospy.init_node("talker", anonymous=True)
pub = rospy.Publisher("chatter", String, queue_size=10)
rate = rospy.Rate(10)  # 10hz
threading.Thread(target=rospy.spin).start()  # Concession for ROS2: spin is always needed
while not rospy.is_shutdown():
    hello_str = "hello world %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    rate.sleep()
```

In ROS1 it provides `rospy`, and in ROS2 it provides matching API classes and functions.

Main behavioral differences of `rospify` in ROS2 vs ROS1:
- `Subscriber.get_num_connections()` always returns 0, as ROS2 does not provide this information
- subscribing with `AnyMsg` needs the publisher existing
- parameter operations always apply to node parameters, as ROS2 has no global parameters
- functions registered in `on_shutdown()` are called after node shutdown, not before
- `rosros.rospify.get_master()` requires local node to be initialized, unlike `rospy.get_master()`
- `rosros.rospify.SubscribeListener` does nothing


### Existing ROS2 package to ROS1/ROS2 package

rosros can also be used as a (mostly) drop-in replacement for rclpy
to make a ROS2 package equally usable under ROS1 - if the code mainly relies on
`rclpy` module and does not go deep into using ROS2 specifics like `builtin_interfaces`.

`rclpy` imports can be replaced with `rosros.rclify`, e.g.

```python
from rosros import rclify as rclpy
from std_msgs.msg import String

class MinimalPublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "chatter", 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def timer_callback(self):
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

rclpy.init()
minimal_publisher = MinimalPublisher()
rclpy.spin(minimal_publisher)
```

In ROS2, `rclify` provides `rclpy`, and in ROS1 it provides
matching API modules and classes and functions.

Covers all the parts of `rclpy.node` that are supportable in ROS1, focused on:
creating nodes, publishing and subscribing topics, providing and invoking services,
working with rates and timers.

Main behavioral differences of `rclify` in ROS1 vs ROS2:
- one single node as always in ROS1
- QoS profiles ignore all options except `depth` (taken as `queue_size`),
  and `durability` equaling `TRANSIENT_LOCAL` (taken as `latch=True`)
- no contexts, no executors, no waitables
- no callback groups, no guard conditions, no middleware events

Methods pertaining to unsupported functionality do nothing and return `None`.
Unsupported arguments in methods are ignored.


Installation
------------

### Using pip

    pip install rosros

This will make the `rosros` library available to Python packages.

Requires ROS Python packages
(ROS1: rospy, roslib, rosservice, genpy;
 ROS2: rclpy, rosidl_parser, rosidl_runtime_py, builtin_interfaces).


### Using catkin

In a ROS1 workspace, under the source directory:

    git clone https://github.com/suurjaak/rosros.git
    cd rosros
    catkin build --this

This will make the `rosros` library available in the ROS1 workspace.


### Using colcon

In a ROS2 workspace, at the workspace root:

    git clone https://github.com/suurjaak/rosros.git src/rosros
    colcon build --packages-select rosros

This will make the `rosros` library available in the ROS2 workspace.


Running tests
-------------

ROS1:

    catkin test rosros

ROS2:

    colcon test --packages-select rosros

To run ROS2 tests with full log output:

    colcon test --packages-select rosros --event-handlers console_direct+ \
                --pytest-args "--capture=tee-sys"

Using pytest, either ROS1 or ROS2 (roscore needs to be running if ROS1):

    cd rosros
    pytest


Dependencies
------------

- pyyaml (https://pypi.org/project/PyYAML)

ROS1:
- genpy
- rosbag
- roslib
- rospy
- rosservice

ROS2:
- builtin_interfaces
- rclpy
- rosidl_parser
- rosidl_runtime_py

ROS2 test dependencies:
- pytest (https://pypi.org/project/pytest)
- pytest-forked (https://pypi.org/project/pytest-forked)

For generating API documentation:

- doxypypy (https://pypi.org/project/doxypypy;
            needs latest master: `pip install git+https://github.com/Feneric/doxypypy`)


Attribution
-----------

This package includes partially copied and modified code from `rclpy` core library,
released under the Apache-2.0 license.
See [doc/3rd-party licences.txt](doc/3rd-party licences.txt) for full details.


License
-------

Copyright (c) 2022 by Erki Suurjaak.
Released as free open source software under the BSD License,
see [LICENSE.md](LICENSE.md) for full details.
