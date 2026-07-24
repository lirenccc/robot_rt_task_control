cmake_minimum_required(VERSION 3.8)
project(robot_navigation_adapters)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_capability_api REQUIRED)
find_package(robot_core_api REQUIRED)
find_package(robot_interfaces REQUIRED)

set(NAV_SOURCES
  src/ros_action_navigation_port.cpp
  src/mock_navigation_port.cpp
)

find_package(nav2_msgs QUIET)
if(nav2_msgs_FOUND)
  list(APPEND NAV_SOURCES src/nav2_navigation_port.cpp)
  add_compile_definitions(ROBOT_HAS_NAV2=1)
  message(STATUS "robot_navigation_adapters: building Nav2NavigationPort (nav2_msgs found)")
else()
  message(STATUS "robot_navigation_adapters: nav2_msgs not found; Nav2 template skipped (install nav2 to enable)")
endif()

add_library(${PROJECT_NAME} SHARED ${NAV_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_action robot_capability_api robot_core_api robot_interfaces)
if(nav2_msgs_FOUND)
  ament_target_dependencies(${PROJECT_NAME} nav2_msgs)
endif()

add_executable(mock_navigation_server src/mock_navigation_server.cpp)
ament_target_dependencies(mock_navigation_server rclcpp rclcpp_action robot_interfaces)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS mock_navigation_server DESTINATION lib/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_action robot_capability_api robot_core_api robot_interfaces)

ament_package()
