# 模块 robot_interfaces：框架通用 ROS 接口（如 RtLoopStats）。
cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

# colcon may pass catkin-only cache variables; reference to silence unused warnings.
if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  set(_robot_interfaces_catkin_legacy "${CATKIN_INSTALL_INTO_PREFIX_ROOT}")
endif()
if(DEFINED CATKIN_SYMLINK_INSTALL)
  set(_robot_interfaces_catkin_legacy_install "${CATKIN_SYMLINK_INSTALL}")
endif()
unset(_robot_interfaces_catkin_legacy)
unset(_robot_interfaces_catkin_legacy_install)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Task.action"
  "action/NavigateToPose.action"
  "action/ExecuteManipulation.action"
  "msg/TaskState.msg"
  "msg/HardwareHealth.msg"
  "msg/RtLoopStats.msg"
  "srv/SetTaskMode.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_package()
