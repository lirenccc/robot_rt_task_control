cmake_minimum_required(VERSION 3.8)
project(robot_hardware_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robot_hardware_api REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/mock_hardware_bus.cpp
  src/skeleton_hardware_bus.cpp
  src/reference_sim_hardware_bus.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} pluginlib robot_hardware_api)

pluginlib_export_plugin_description_file(robot_hardware_api plugins/robot_hardware_plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY plugins/ DESTINATION share/${PROJECT_NAME}/plugins)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(pluginlib robot_hardware_api)

ament_package()
