#include "robot_runtime/provider_registry.hpp"

#include <pluginlib/class_loader.hpp>

#include "robot_manipulation_adapters/follow_joint_trajectory_port.hpp"
#include "robot_manipulation_adapters/manipulation_ports.hpp"
#include "robot_navigation_adapters/navigation_ports.hpp"
#ifdef ROBOT_HAS_NAV2
#include "robot_navigation_adapters/nav2_navigation_port.hpp"
#endif
#include "robot_safety/velocity_limit_policy.hpp"
#include "robot_task/planner.hpp"
#include "robot_task/yaml_graph_planner.hpp"

namespace robot_runtime
{

ProviderRegistry & ProviderRegistry::builtins()
{
  static ProviderRegistry registry = [] {
    ProviderRegistry r;
    register_builtin_providers(r);
    return r;
  }();
  return registry;
}

void ProviderRegistry::register_navigation(const std::string & type, NavigationFactoryFn fn)
{
  navigation_[type] = std::move(fn);
}

void ProviderRegistry::register_manipulation(const std::string & type, ManipulationFactoryFn fn)
{
  manipulation_[type] = std::move(fn);
}

void ProviderRegistry::register_planner(const std::string & type, PlannerFactoryFn fn)
{
  planners_[type] = std::move(fn);
}

void ProviderRegistry::register_safety_policy(const std::string & name, SafetyPolicyFactoryFn fn)
{
  safety_[name] = std::move(fn);
}

robot_core_api::Result<std::shared_ptr<robot_capability_api::INavigationPort>>
ProviderRegistry::make_navigation(const std::string & type, const ProviderContext & ctx) const
{
  const auto it = navigation_.find(type);
  if (it == navigation_.end()) {
    return robot_core_api::Result<std::shared_ptr<robot_capability_api::INavigationPort>>::failure(
      robot_core_api::ErrorCode::InvalidConfiguration,
      "Unknown navigation provider type: " + type);
  }
  return it->second(ctx);
}

robot_core_api::Result<std::shared_ptr<robot_capability_api::IManipulationPort>>
ProviderRegistry::make_manipulation(const std::string & type, const ProviderContext & ctx) const
{
  const auto it = manipulation_.find(type);
  if (it == manipulation_.end()) {
    return robot_core_api::Result<std::shared_ptr<robot_capability_api::IManipulationPort>>::failure(
      robot_core_api::ErrorCode::InvalidConfiguration,
      "Unknown manipulation provider type: " + type);
  }
  return it->second(ctx);
}

robot_core_api::Result<std::shared_ptr<robot_task::ITaskPlanner>>
ProviderRegistry::make_planner(const std::string & type, const ProviderContext & ctx) const
{
  const auto it = planners_.find(type);
  if (it == planners_.end()) {
    return robot_core_api::Result<std::shared_ptr<robot_task::ITaskPlanner>>::failure(
      robot_core_api::ErrorCode::InvalidConfiguration,
      "Unknown planner type: " + type);
  }
  return it->second(ctx);
}

robot_core_api::Result<std::shared_ptr<robot_safety::ISafetyPolicy>>
ProviderRegistry::make_safety_policy(const std::string & name, const ProviderContext & ctx) const
{
  const auto it = safety_.find(name);
  if (it != safety_.end()) {
    return it->second(ctx, name);
  }

  try {
    pluginlib::ClassLoader<robot_safety::ISafetyPolicy> loader(
      "robot_safety", "robot_safety::ISafetyPolicy");
    auto policy = loader.createSharedInstance(name);
    if (auto * velocity = dynamic_cast<robot_safety::VelocityLimitPolicy *>(policy.get())) {
      velocity->set_limits(ctx.profile.limits);
    }
    return robot_core_api::Result<std::shared_ptr<robot_safety::ISafetyPolicy>>::success(
      std::move(policy));
  } catch (const pluginlib::PluginlibException & ex) {
    return robot_core_api::Result<std::shared_ptr<robot_safety::ISafetyPolicy>>::failure(
      robot_core_api::ErrorCode::InvalidConfiguration,
      std::string("Failed to load safety policy '") + name + "': " + ex.what());
  }
}

void register_builtin_providers(ProviderRegistry & registry)
{
  registry.register_navigation(
    "mock",
    [](const ProviderContext &) {
      return robot_core_api::Result<std::shared_ptr<robot_capability_api::INavigationPort>>::success(
        std::make_shared<robot_navigation_adapters::MockNavigationPort>());
    });

  registry.register_navigation(
    "ros_action",
    [](const ProviderContext & ctx) {
      if (!ctx.node) {
        return robot_core_api::Result<std::shared_ptr<robot_capability_api::INavigationPort>>::
          failure(
          robot_core_api::ErrorCode::InvalidConfiguration,
          "Node required for navigation provider: ros_action");
      }
      return robot_core_api::Result<std::shared_ptr<robot_capability_api::INavigationPort>>::success(
        std::make_shared<robot_navigation_adapters::RosActionNavigationPort>(
          ctx.node, ctx.runtime.navigation.endpoint));
    });

  registry.register_navigation(
    "nav2",
    [](const ProviderContext & ctx) {
      if (!ctx.node) {
        return robot_core_api::Result<std::shared_ptr<robot_capability_api::INavigationPort>>::
          failure(
          robot_core_api::ErrorCode::InvalidConfiguration,
          "Node required for navigation provider: nav2");
      }
#ifdef ROBOT_HAS_NAV2
      return robot_core_api::Result<std::shared_ptr<robot_capability_api::INavigationPort>>::success(
        std::make_shared<robot_navigation_adapters::Nav2NavigationPort>(
          ctx.node,
          ctx.runtime.navigation.endpoint.empty()
            ? "navigate_to_pose" : ctx.runtime.navigation.endpoint));
#else
      return robot_core_api::Result<std::shared_ptr<robot_capability_api::INavigationPort>>::failure(
        robot_core_api::ErrorCode::Unavailable,
        "nav2 provider requested but nav2_msgs was not available at build time");
#endif
    });

  registry.register_manipulation(
    "mock",
    [](const ProviderContext &) {
      return robot_core_api::Result<std::shared_ptr<robot_capability_api::IManipulationPort>>::
        success(std::make_shared<robot_manipulation_adapters::MockManipulationPort>());
    });

  registry.register_manipulation(
    "ros_action",
    [](const ProviderContext & ctx) {
      if (!ctx.node) {
        return robot_core_api::Result<std::shared_ptr<robot_capability_api::IManipulationPort>>::
          failure(
          robot_core_api::ErrorCode::InvalidConfiguration,
          "Node required for ros_action manipulation");
      }
      return robot_core_api::Result<std::shared_ptr<robot_capability_api::IManipulationPort>>::
        success(
        std::make_shared<robot_manipulation_adapters::RosActionManipulationPort>(
          ctx.node, ctx.runtime.manipulation.endpoint));
    });

  auto traj_fn = [](const ProviderContext & ctx) {
    if (!ctx.node) {
      return robot_core_api::Result<std::shared_ptr<robot_capability_api::IManipulationPort>>::
        failure(
        robot_core_api::ErrorCode::InvalidConfiguration,
        "Node required for trajectory manipulation");
    }
    return robot_core_api::Result<std::shared_ptr<robot_capability_api::IManipulationPort>>::
      success(
      std::make_shared<robot_manipulation_adapters::FollowJointTrajectoryManipulationPort>(
        ctx.node,
        ctx.runtime.manipulation.endpoint.empty()
          ? "/arm_controller/follow_joint_trajectory" : ctx.runtime.manipulation.endpoint));
  };
  registry.register_manipulation("follow_joint_trajectory", traj_fn);
  registry.register_manipulation("moveit", traj_fn);

  registry.register_planner(
    "simple",
    [](const ProviderContext & ctx) {
      return robot_core_api::Result<std::shared_ptr<robot_task::ITaskPlanner>>::success(
        std::make_shared<robot_task::SimpleTaskPlanner>(ctx.profile));
    });

  registry.register_planner(
    "yaml_graph",
    [](const ProviderContext & ctx) {
      const std::string dir = ctx.runtime.planner.tasks_dir.empty()
        ? "tasks" : ctx.runtime.planner.tasks_dir;
      return robot_core_api::Result<std::shared_ptr<robot_task::ITaskPlanner>>::success(
        std::make_shared<robot_task::YamlGraphPlanner>(ctx.profile, dir));
    });

  auto velocity_fn = [](const ProviderContext & ctx, const std::string &) {
    auto policy = std::make_shared<robot_safety::VelocityLimitPolicy>(ctx.profile.limits);
    return robot_core_api::Result<std::shared_ptr<robot_safety::ISafetyPolicy>>::success(
      std::move(policy));
  };
  registry.register_safety_policy("robot_safety/VelocityLimitPolicy", velocity_fn);
  registry.register_safety_policy("VelocityLimitPolicy", velocity_fn);
}

}  // namespace robot_runtime
