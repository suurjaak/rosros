\internal  Entrypoint README for doxygen  \endinternal

Simple unified interface to ROS1 / ROS2 Python API.

Auto-detects ROS version from run-time environment, and provides
equivalent functionality in either ROS version.

Main use-cases:
- helper library for working with ROS messages and data types
- replacement for ROS1 / ROS2 Python API
- drop-in replacement for `rospy` to convert a ROS1 package to a ROS1/ROS2 package
- drop-in replacement for `rclpy` to convert a ROS2 package to a ROS1/ROS2 package




| Module Index                                                                                         ||
| --------------------------- | -------------------------------------------------------------------------
| [rosros](@ref rosros.core)  | core operations for ROS system and nodes
| rosros.api                  | Utilities for ROS built-in types, and message and service types
| rosros.parsing              | Utilities for ROS message definition texts
| rosros.rclify               | Provides ROS2 API facade for ROS1, or imports `rclpy` if ROS2 environment
| rosros.rospify              | Provides ROS1 API facade for ROS2, or imports `rospy` if ROS1 environment
| rosros.util                 | Common utility classes and functions


Internal Modules                                                                                  
----------------     

|                             ||
|-----------------------------|------------------------------------------------------------------
| rosros.patch                | Patches ROS1 classes with ROS2-compatible members, and vice versa
| rosros.ros1                 | ROS1 core interface
| rosros.ros2                 | ROS2 core interface
