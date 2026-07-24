/**
 * @brief 模块 robot_runtime：组合根（RuntimeBuilder、配置加载、适配器工厂）。
 *
 * 装配 profile / 能力端口 / 安全门，作为非实时运行时入口。
 */

#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_capability_api/ports.hpp"
#include "robot_core_api/fault.hpp"
#include "robot_core_api/result.hpp"
#include "robot_profile/robot_profile.hpp"
#include "robot_runtime/health_monitor.hpp"
#include "robot_runtime/runtime_config.hpp"
#include "robot_runtime/runtime_lifecycle.hpp"
#include "robot_safety/safety_gate.hpp"
#include "robot_task/task_orchestrator.hpp"

namespace robot_runtime
{

struct RobotRuntime
{
  robot_profile::RobotProfile profile;
  RuntimeConfig runtime_config;
  std::shared_ptr<robot_core_api::FaultManager> faults;
  std::shared_ptr<robot_capability_api::INavigationPort> navigation;
  std::shared_ptr<robot_capability_api::IManipulationPort> manipulation;
  std::shared_ptr<robot_safety::SafetyGate> safety_gate;
  std::shared_ptr<robot_task::TaskOrchestrator> orchestrator;
  std::shared_ptr<HealthMonitor> health;
  RuntimeLifecycle lifecycle;

  robot_core_api::Status configure();
  robot_core_api::Status activate();
  robot_core_api::Status deactivate();
  robot_core_api::Status cleanup();
};

/// Unique composition root. Business modules must not load YAML or plugins themselves.
class RuntimeBuilder
{
public:
  robot_core_api::Result<RobotRuntime> build_from_files(
    const std::string & profile_path,
    const std::string & runtime_path,
    const rclcpp::Node::SharedPtr & node) const;

  robot_core_api::Result<RobotRuntime> build(
    const robot_profile::RobotProfile & profile,
    const RuntimeConfig & runtime,
    const rclcpp::Node::SharedPtr & node) const;
};

}  // namespace robot_runtime
