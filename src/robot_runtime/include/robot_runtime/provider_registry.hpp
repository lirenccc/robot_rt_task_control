#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include "robot_capability_api/ports.hpp"
#include "robot_core_api/result.hpp"
#include "robot_profile/robot_profile.hpp"
#include "robot_runtime/runtime_config.hpp"
#include "robot_safety/safety_policy.hpp"
#include "robot_task/planner.hpp"

namespace robot_runtime
{

struct ProviderContext
{
  robot_profile::RobotProfile profile;
  RuntimeConfig runtime;
  rclcpp::Node::SharedPtr node;
};

using NavigationFactoryFn = std::function<robot_core_api::Result<
    std::shared_ptr<robot_capability_api::INavigationPort>>(const ProviderContext &)>;
using ManipulationFactoryFn = std::function<robot_core_api::Result<
    std::shared_ptr<robot_capability_api::IManipulationPort>>(const ProviderContext &)>;
using PlannerFactoryFn = std::function<robot_core_api::Result<
    std::shared_ptr<robot_task::ITaskPlanner>>(const ProviderContext &)>;
using SafetyPolicyFactoryFn = std::function<robot_core_api::Result<
    std::shared_ptr<robot_safety::ISafetyPolicy>>(
    const ProviderContext &, const std::string & plugin_name)>;

/// String-keyed registries used by RuntimeBuilder (Nav/Manip stay Adapter-based).
class ProviderRegistry
{
public:
  static ProviderRegistry & builtins();

  void register_navigation(const std::string & type, NavigationFactoryFn fn);
  void register_manipulation(const std::string & type, ManipulationFactoryFn fn);
  void register_planner(const std::string & type, PlannerFactoryFn fn);
  void register_safety_policy(const std::string & name, SafetyPolicyFactoryFn fn);

  robot_core_api::Result<std::shared_ptr<robot_capability_api::INavigationPort>>
  make_navigation(const std::string & type, const ProviderContext & ctx) const;

  robot_core_api::Result<std::shared_ptr<robot_capability_api::IManipulationPort>>
  make_manipulation(const std::string & type, const ProviderContext & ctx) const;

  robot_core_api::Result<std::shared_ptr<robot_task::ITaskPlanner>>
  make_planner(const std::string & type, const ProviderContext & ctx) const;

  robot_core_api::Result<std::shared_ptr<robot_safety::ISafetyPolicy>>
  make_safety_policy(const std::string & name, const ProviderContext & ctx) const;

private:
  std::unordered_map<std::string, NavigationFactoryFn> navigation_;
  std::unordered_map<std::string, ManipulationFactoryFn> manipulation_;
  std::unordered_map<std::string, PlannerFactoryFn> planners_;
  std::unordered_map<std::string, SafetyPolicyFactoryFn> safety_;
};

void register_builtin_providers(ProviderRegistry & registry);

}  // namespace robot_runtime
