cmake_minimum_required(VERSION 3.8)
project(robot_ethercat_adapters)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robot_hardware_api REQUIRED)

find_package(ethercat_master_ecmaster QUIET)
find_package(ethercat_master_igh QUIET)

set(BUILT_ANY FALSE)

# Separate libraries: both masters export ethercat_joint::* and cannot link together.
if(ethercat_master_ecmaster_FOUND)
  set(BUILT_ANY TRUE)
  add_library(robot_ethercat_adapters_ecmaster SHARED
    src/ecmaster_hardware_bus.cpp)
  target_include_directories(robot_ethercat_adapters_ecmaster PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(robot_ethercat_adapters_ecmaster pluginlib robot_hardware_api)
  target_link_libraries(robot_ethercat_adapters_ecmaster
    ethercat_master_ecmaster::ethercat_master_ecmaster)
  pluginlib_export_plugin_description_file(
    robot_hardware_api plugins/ecmaster_hardware_bus.xml)
  install(TARGETS robot_ethercat_adapters_ecmaster
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  message(STATUS "robot_ethercat_adapters: building EcMasterHardwareBus")
else()
  message(STATUS
    "robot_ethercat_adapters: ethercat_master_ecmaster not found; EcMasterHardwareBus skipped")
endif()

if(ethercat_master_igh_FOUND)
  set(BUILT_ANY TRUE)
  add_library(robot_ethercat_adapters_igh SHARED
    src/igh_hardware_bus.cpp)
  target_include_directories(robot_ethercat_adapters_igh PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(robot_ethercat_adapters_igh pluginlib robot_hardware_api)
  target_link_libraries(robot_ethercat_adapters_igh
    ethercat_master_igh::ethercat_master_igh)
  pluginlib_export_plugin_description_file(
    robot_hardware_api plugins/igh_hardware_bus.xml)
  install(TARGETS robot_ethercat_adapters_igh
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  message(STATUS "robot_ethercat_adapters: building IghHardwareBus")
else()
  message(STATUS
    "robot_ethercat_adapters: ethercat_master_igh not found; IghHardwareBus skipped")
endif()

if(BUILT_ANY)
  install(DIRECTORY include/ DESTINATION include)
  install(DIRECTORY plugins/ DESTINATION share/${PROJECT_NAME}/plugins)
  ament_export_include_directories(include)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
else()
  message(WARNING
    "robot_ethercat_adapters: no EtherCAT master packages found; "
    "package installs metadata only (no HardwareBus plugins).")
endif()

ament_export_dependencies(pluginlib robot_hardware_api)
ament_package()
