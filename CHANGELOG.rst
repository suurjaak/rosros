^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.3 (2023-12-08)
------------------
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
