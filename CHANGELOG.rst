^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2023-12-06)
------------------
* add rosros.wait_for_publisher(), wait_for_subscriber() and wait_for_service()
* fix rosros.get_services() raising error on node-parameter in ROS2

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
