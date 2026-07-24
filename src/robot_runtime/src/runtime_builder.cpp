#include "robot_runtime/runtime_builder.hpp"

#include "robot_profile/loader.hpp"
#include "robot_runtime/provider_registry.hpp"
#include "robot_runtime/runtime_config_loader.hpp"

namespace robot_runtime
{

robot_core_api::Status RobotRuntime::configure()
{
  return lifecycle.configure_all();
}

robot_core_api::Status RobotRuntime::activate()
{
  return lifecycle.activate_all();
}

robot_core_api::Status RobotRuntime::deactivate()
{
  return lifecycle.deactivate_all();
}

robot_core_api::Status RobotRuntime::cleanup()
{
  return lifecycle.cleanup_all();
}

robot_core_api::Result<RobotRuntime> RuntimeBuilder::build_from_files(
  const std::string & profile_path,
  const std::string & runtime_path,
  const rclcpp::Node::SharedPtr & node) const
{
  robot_profile::ProfileLoader profile_loader;
  auto profile = profile_loader.load_file(profile_path);
  if (!profile.ok()) {
    return robot_core_api::Result<RobotRuntime>::failure(profile.status());
  }

  RuntimeConfigLoader runtime_loader;
  auto runtime = runtime_loader.load_file(runtime_path);
  if (!runtime.ok()) {
    return robot_core_api::Result<RobotRuntime>::failure(runtime.status());
  }

  return build(profile.value(), runtime.value(), node);
}

robot_core_api::Result<RobotRuntime> RuntimeBuilder::build(
  const robot_profile::RobotProfile & profile,
  const RuntimeConfig & runtime,
  const rclcpp::Node::SharedPtr & node) const
{
  using robot_core_api::ErrorCode;
  using robot_core_api::Result;

  RobotRuntime out;
  out.profile = profile;
  out.runtime_config = runtime;
  out.faults = std::make_shared<robot_core_api::FaultManager>();

  ProviderContext ctx;
  ctx.profile = profile;
  ctx.runtime = runtime;
  ctx.node = node;

  auto & registry = ProviderRegistry::builtins();

  auto nav = registry.make_navigation(runtime.navigation.type, ctx);
  if (!nav.ok()) {
    return Result<RobotRuntime>::failure(nav.status());
  }
  out.navigation = nav.value();

  auto manip = registry.make_manipulation(runtime.manipulation.type, ctx);
  if (!manip.ok()) {
    return Result<RobotRuntime>::failure(manip.status());
  }
  out.manipulation = manip.value();

  out.safety_gate = std::make_shared<robot_safety::SafetyGate>();
  std::vector<std::string> policies = runtime.safety_policies;
  if (policies.empty()) {
    policies.push_back("robot_safety/VelocityLimitPolicy");
  }
  for (const auto & name : policies) {
    auto policy = registry.make_safety_policy(name, ctx);
    if (!policy.ok()) {
      return Result<RobotRuntime>::failure(policy.status());
    }
    out.safety_gate->add_policy(policy.value());
  }

  const std::string planner_type =
    runtime.planner.type.empty() ? "simple" : runtime.planner.type;
  auto planner = registry.make_planner(planner_type, ctx);
  if (!planner.ok()) {
    return Result<RobotRuntime>::failure(planner.status());
  }

  out.orchestrator = std::make_shared<robot_task::TaskOrchestrator>(
    out.navigation, out.manipulation, out.safety_gate, out.faults, planner.value());

  out.health = std::make_shared<HealthMonitor>(
    node, out.faults, out.navigation, out.manipulation);
  out.lifecycle.add(out.health);

  (void)runtime.hardware_base;
  return Result<RobotRuntime>::success(std::move(out));
}

}  // namespace robot_runtime
