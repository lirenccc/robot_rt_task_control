cmake_minimum_required(VERSION 3.8)
project(robot_ros2_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(robot_hardware_api REQUIRED)
find_package(robot_hardware_plugins REQUIRED)
find_package(robot_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rt_control_loop.cpp
  src/rt_mobile_manipulator_system.cpp
  src/safety_limiter.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  robot_hardware_api
  robot_hardware_plugins
  robot_interfaces
)

pluginlib_export_plugin_description_file(hardware_interface plugins/robot_ros2_control.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY plugins/
  DESTINATION share/${PROJECT_NAME}/plugins
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  robot_hardware_api
  robot_hardware_plugins
  robot_interfaces
)

ament_package()
