# 模块 robot_testkit：契约测试与 Mock 资产（与生产相同的插件加载路径）。
cmake_minimum_required(VERSION 3.8)
project(robot_testkit)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_core_api REQUIRED)
find_package(robot_hardware_api REQUIRED)
find_package(robot_hardware_plugins REQUIRED)
find_package(robot_profile REQUIRED)
find_package(robot_safety REQUIRED)
find_package(robot_capability_api REQUIRED)
find_package(robot_navigation_adapters REQUIRED)
find_package(robot_manipulation_adapters REQUIRED)
find_package(robot_runtime REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(robot_ros2_control REQUIRED)
find_package(robot_task REQUIRED)
find_package(robot_bringup REQUIRED)
find_package(nav2_msgs QUIET)
if(nav2_msgs_FOUND)
  add_compile_definitions(ROBOT_HAS_NAV2=1)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_profile_validation test/test_profile_validation.cpp)
  ament_target_dependencies(test_profile_validation robot_profile robot_core_api)

  ament_add_gtest(test_hardware_plugin_discovery test/test_hardware_plugin_discovery.cpp)
  ament_target_dependencies(test_hardware_plugin_discovery
    pluginlib robot_hardware_api robot_hardware_plugins)

  ament_add_gtest(test_fault_manager test/test_fault_manager.cpp)
  ament_target_dependencies(test_fault_manager robot_core_api)

  ament_add_gtest(test_safety_gate test/test_safety_gate.cpp)
  ament_target_dependencies(test_safety_gate robot_safety robot_profile robot_core_api)

  ament_add_gtest(test_capability_mock_contract test/test_capability_mock_contract.cpp)
  ament_target_dependencies(test_capability_mock_contract
    robot_capability_api robot_navigation_adapters robot_manipulation_adapters robot_core_api)

  ament_add_gtest(test_manipulation_joint_goal test/test_manipulation_joint_goal.cpp)
  ament_target_dependencies(test_manipulation_joint_goal robot_capability_api robot_core_api)

  ament_add_gtest(test_nav2_adapter_present test/test_nav2_adapter_present.cpp)
  ament_target_dependencies(test_nav2_adapter_present robot_navigation_adapters)
  if(nav2_msgs_FOUND)
    ament_target_dependencies(test_nav2_adapter_present nav2_msgs)
  endif()

  ament_add_gtest(test_health_monitor_rt_fault test/test_health_monitor_rt_fault.cpp)
  ament_target_dependencies(test_health_monitor_rt_fault
    rclcpp robot_runtime robot_core_api robot_interfaces)

  ament_add_gtest(test_soft_rt_baseline test/test_soft_rt_baseline.cpp)
  ament_target_dependencies(test_soft_rt_baseline
    pluginlib robot_hardware_api robot_hardware_plugins robot_ros2_control)

  ament_add_gtest(test_provider_registry_and_dsl test/test_provider_registry_and_dsl.cpp)
  ament_target_dependencies(test_provider_registry_and_dsl
    ament_index_cpp pluginlib robot_core_api robot_profile robot_safety
    robot_capability_api robot_navigation_adapters robot_manipulation_adapters
    robot_runtime robot_task robot_bringup)
  if(nav2_msgs_FOUND)
    ament_target_dependencies(test_provider_registry_and_dsl nav2_msgs)
  endif()

  add_launch_test(test/test_task_stack_inprocess.test.py
    TIMEOUT 60
  )
endif()

ament_package()
