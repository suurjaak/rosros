^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.6 (2024-11-20)
------------------
* fix rclify error on loading node parameter overrides (issue #1)
* fix compatibility with ROS 2 Jazzy (issue #2)
* fix parsing and hashing message type definitions with leading whitespace

0.2.5 (2024-01-30)
------------------
* make api.dict_to_message() support nested ROS messages in dictionary

0.2.4 (2023-12-29)
------------------
* fix api.to_duration()

0.2.3 (2023-12-10)
------------------
* add rosros.on_shutdown()
* add rosros.sleep()
* add rosros.get_ros_version()
* add optional reason-parameter to rosros.shutdown()
* support giving message class or typename in api.dict_to_message()
* add test for core functions
* fix rospify.sleep() not returning immediately for argument value <=0

0.2.2 (2023-12-07)
------------------
* add rosros.wait_for_publisher(), wait_for_subscriber() and wait_for_service()
* allow service callbacks to return None
* fix service calls raising error in ROS2
* fix rosros.get_services() raising error on node-parameter in ROS2
* fix rosros.get_param_names() returning params dictionary instead of list
* fix rosros.set_param() raising error on no descriptor in ROS2
* fix util.get_arity() and wrap_arity() raising for built-in functions
* fix rosros.spin_once() raising error on negative timeout in ROS1
* fix api.get_message_type() not returning value for service in ROS1
* fix throttling log messages again

0.2.1 (2023-12-06)
------------------
* add api.to_duration()
* add default fallback parameter to api.get_message_value()
* fix api.dict_to_message() erroring on temporal types in dict
* fix util.get_arity() counting some parameters twice if counting both positional and keyword arguments
* fix api.time_message() not using clock_type parameter in ROS2
* fix patching floordiv to ROS2 duration class
* fix throttling log messages, use monotonic time

0.2.0 (2023-02-28)
-------------------
* support dictionaries in publish and service calls
* return node object from rosros.init_node() in ROS2

0.1.0 (2022-10-30)
-------------------
* first release
