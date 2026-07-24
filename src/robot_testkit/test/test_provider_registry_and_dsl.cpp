#include <gtest/gtest.h>

#include <atomic>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_loader.hpp>

#include "robot_core_api/fault.hpp"
#include "robot_manipulation_adapters/manipulation_ports.hpp"
#include "robot_navigation_adapters/navigation_ports.hpp"
#include "robot_profile/robot_profile.hpp"
#include "robot_runtime/provider_registry.hpp"
#include "robot_runtime/runtime_config.hpp"
#include "robot_safety/safety_gate.hpp"
#include "robot_safety/velocity_limit_policy.hpp"
#include "robot_task/task_orchestrator.hpp"
#include "robot_task/yaml_graph_planner.hpp"

namespace
{

robot_profile::RobotProfile make_profile()
{
  robot_profile::RobotProfile p;
  p.schema_version = 1;
  p.robot.model = "test";
  p.capabilities.navigation = true;
  p.capabilities.manipulation = true;
  p.frames.map = "map";
  p.frames.base = "base_link";
  p.limits.max_linear_velocity = 1.0;
  p.limits.max_angular_velocity = 1.0;
  return p;
}

std::string tasks_dir()
{
  return ament_index_cpp::get_package_share_directory("robot_bringup") + "/config/tasks";
}

}  // namespace

TEST(ProviderRegistry, UnknownNavigationTypeFails)
{
  robot_runtime::ProviderContext ctx;
  ctx.profile = make_profile();
  auto result = robot_runtime::ProviderRegistry::builtins().make_navigation("nope", ctx);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, robot_core_api::ErrorCode::InvalidConfiguration);
}

TEST(ProviderRegistry, MockProvidersBuild)
{
  robot_runtime::ProviderContext ctx;
  ctx.profile = make_profile();
  auto & reg = robot_runtime::ProviderRegistry::builtins();
  auto nav = reg.make_navigation("mock", ctx);
  auto manip = reg.make_manipulation("mock", ctx);
  auto planner = reg.make_planner("simple", ctx);
  ASSERT_TRUE(nav.ok()) << nav.status().message;
  ASSERT_TRUE(manip.ok()) << manip.status().message;
  ASSERT_TRUE(planner.ok()) << planner.status().message;
}

TEST(YamlGraphPlanner, LoadsDemoPick)
{
  robot_task::YamlGraphPlanner planner(make_profile(), tasks_dir());
  robot_task::TaskRequest req;
  req.instruction = "graph:demo_pick";
  auto graph = planner.plan(req);
  ASSERT_TRUE(graph.ok()) << graph.status().message;
  EXPECT_EQ(graph.value().steps.size(), 2u);
}

TEST(YamlGraphPlanner, MissingFileFails)
{
  robot_task::YamlGraphPlanner planner(make_profile(), "/tmp");
  robot_task::TaskRequest req;
  req.instruction = "graph:does_not_exist";
  auto graph = planner.plan(req);
  EXPECT_FALSE(graph.ok());
}

TEST(YamlGraphPlanner, OrchestratorRunsMockGraph)
{
  const auto profile = make_profile();
  auto nav = std::make_shared<robot_navigation_adapters::MockNavigationPort>();
  auto manip = std::make_shared<robot_manipulation_adapters::MockManipulationPort>();
  auto gate = std::make_shared<robot_safety::SafetyGate>();
  gate->add_policy(std::make_shared<robot_safety::VelocityLimitPolicy>(profile.limits));
  auto faults = std::make_shared<robot_core_api::FaultManager>();
  auto planner = std::make_shared<robot_task::YamlGraphPlanner>(profile, tasks_dir());
  robot_task::TaskOrchestrator orch(nav, manip, gate, faults, planner);

  robot_task::TaskRequest req;
  req.instruction = "graph:demo_pick";
  std::atomic<bool> cancel{false};
  auto status = orch.execute(req, nullptr, cancel);
  EXPECT_TRUE(status.ok()) << status.message;
}

TEST(SafetyPluginlib, LoadsVelocityLimitPolicy)
{
  pluginlib::ClassLoader<robot_safety::ISafetyPolicy> loader(
    "robot_safety", "robot_safety::ISafetyPolicy");
  auto policy = loader.createSharedInstance("robot_safety/VelocityLimitPolicy");
  ASSERT_NE(policy, nullptr);
  auto * vel = dynamic_cast<robot_safety::VelocityLimitPolicy *>(policy.get());
  ASSERT_NE(vel, nullptr);
  vel->set_limits(make_profile().limits);
  robot_safety::RobotCommand cmd;
  cmd.kind = robot_safety::CommandKind::Navigate;
  cmd.requested_linear_velocity = 0.1;
  cmd.requested_angular_velocity = 0.1;
  robot_safety::RobotState state;
  state.state_known = true;
  EXPECT_TRUE(policy->admit(cmd, state).allowed());
}

TEST(SafetyPluginlib, MissingPluginFails)
{
  robot_runtime::ProviderContext ctx;
  ctx.profile = make_profile();
  auto result = robot_runtime::ProviderRegistry::builtins().make_safety_policy(
    "robot_safety/DoesNotExist", ctx);
  EXPECT_FALSE(result.ok());
}
