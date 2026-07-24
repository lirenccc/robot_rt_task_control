cmake_minimum_required(VERSION 3.8)
project(robot_manipulation_adapters)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_capability_api REQUIRED)
find_package(robot_core_api REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ros_action_manipulation_port.cpp
  src/mock_manipulation_port.cpp
  src/follow_joint_trajectory_port.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  control_msgs rclcpp rclcpp_action robot_capability_api robot_core_api
  robot_interfaces trajectory_msgs)

add_executable(mock_manipulation_server src/mock_manipulation_server.cpp)
ament_target_dependencies(mock_manipulation_server rclcpp rclcpp_action robot_interfaces)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS mock_manipulation_server DESTINATION lib/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  control_msgs rclcpp rclcpp_action robot_capability_api robot_core_api
  robot_interfaces trajectory_msgs)

ament_package()
