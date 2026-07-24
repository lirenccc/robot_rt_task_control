cmake_minimum_required(VERSION 3.8)
project(robot_runtime)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_capability_api REQUIRED)
find_package(robot_core_api REQUIRED)
find_package(robot_hardware_api REQUIRED)
find_package(robot_manipulation_adapters REQUIRED)
find_package(robot_navigation_adapters REQUIRED)
find_package(robot_profile REQUIRED)
find_package(robot_safety REQUIRED)
find_package(robot_task REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav2_msgs QUIET)
if(nav2_msgs_FOUND)
  add_compile_definitions(ROBOT_HAS_NAV2=1)
endif()

add_library(${PROJECT_NAME} SHARED
  src/runtime_config_loader.cpp
  src/runtime_builder.cpp
  src/runtime_lifecycle.cpp
  src/health_monitor.cpp
  src/provider_registry.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  pluginlib rclcpp robot_capability_api robot_core_api robot_hardware_api
  robot_manipulation_adapters robot_navigation_adapters robot_profile
  robot_safety robot_task robot_interfaces)
if(nav2_msgs_FOUND)
  ament_target_dependencies(${PROJECT_NAME} nav2_msgs)
endif()
target_link_libraries(${PROJECT_NAME} yaml-cpp)

add_executable(task_orchestrator_node src/task_orchestrator_node.cpp)
target_link_libraries(task_orchestrator_node ${PROJECT_NAME})
ament_target_dependencies(task_orchestrator_node
  pluginlib rclcpp rclcpp_action robot_capability_api robot_core_api
  robot_hardware_api robot_manipulation_adapters robot_navigation_adapters
  robot_profile robot_safety robot_task robot_interfaces)
if(nav2_msgs_FOUND)
  ament_target_dependencies(task_orchestrator_node nav2_msgs)
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS task_orchestrator_node DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  pluginlib rclcpp robot_capability_api robot_core_api robot_hardware_api
  robot_manipulation_adapters robot_navigation_adapters robot_profile
  robot_safety robot_task yaml-cpp robot_interfaces)
if(nav2_msgs_FOUND)
  ament_export_dependencies(nav2_msgs)
endif()

ament_package()
