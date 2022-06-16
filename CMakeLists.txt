cmake_minimum_required(VERSION 3.8)
project(rosros)

if($ENV{ROS_VERSION} EQUAL "1")
  find_package(catkin REQUIRED COMPONENTS
    genpy roslib rospy rosservice
  )

  catkin_python_setup()

  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(std_srvs REQUIRED)

    catkin_add_nosetests(test/test_api.py)
    add_rostest(test/test_rclify.launch)
    add_rostest(test/test_rospify.launch)
    add_rostest(test/test_topics.launch)
    add_rostest(test/test_services.launch)
  endif()

  catkin_package(CATKIN_DEPENDS genpy roslib rospy)
else()
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(rclpy REQUIRED)
  find_package(rosidl_parser REQUIRED)
  find_package(rosidl_runtime_py REQUIRED)

  find_package(ament_lint_auto REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(std_srvs REQUIRED)

    ament_add_pytest_test(test_api test/test_api.py)
    ament_add_pytest_test(test_rclify test/test_rclify.py)
    ament_add_pytest_test(test_rospify test/test_rospify.py)
    ament_add_pytest_test(test_topics test/test_topics.py)
    ament_add_pytest_test(test_services test/test_services.py)
  endif()

  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
  ament_package()
endif()
