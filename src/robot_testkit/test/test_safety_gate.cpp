#include <gtest/gtest.h>

#include "robot_profile/robot_profile.hpp"
#include "robot_safety/safety_gate.hpp"
#include "robot_safety/velocity_limit_policy.hpp"

TEST(SafetyGate, DefaultDenyWithoutPolicies)
{
  robot_safety::SafetyGate gate;
  robot_safety::RobotCommand cmd;
  cmd.kind = robot_safety::CommandKind::Navigate;
  robot_safety::RobotState state;
  state.state_known = true;
  const auto decision = gate.admit(cmd, state);
  EXPECT_FALSE(decision.allowed());
}

TEST(SafetyGate, UnknownStateDenied)
{
  robot_safety::SafetyGate gate;
  robot_profile::Limits limits;
  limits.max_linear_velocity = 1.0;
  limits.max_angular_velocity = 1.0;
  gate.add_policy(std::make_shared<robot_safety::VelocityLimitPolicy>(limits));

  robot_safety::RobotCommand cmd;
  cmd.kind = robot_safety::CommandKind::Navigate;
  robot_safety::RobotState state;
  state.state_known = false;
  EXPECT_FALSE(gate.admit(cmd, state).allowed());
}

TEST(SafetyGate, VelocityLimitBlocksExcess)
{
  robot_safety::SafetyGate gate;
  robot_profile::Limits limits;
  limits.max_linear_velocity = 1.0;
  limits.max_angular_velocity = 1.0;
  gate.add_policy(std::make_shared<robot_safety::VelocityLimitPolicy>(limits));

  robot_safety::RobotCommand cmd;
  cmd.kind = robot_safety::CommandKind::Navigate;
  cmd.requested_linear_velocity = 2.0;
  robot_safety::RobotState state;
  state.state_known = true;
  EXPECT_FALSE(gate.admit(cmd, state).allowed());
}

TEST(SafetyGate, AllowsWithinLimits)
{
  robot_safety::SafetyGate gate;
  robot_profile::Limits limits;
  limits.max_linear_velocity = 1.0;
  limits.max_angular_velocity = 1.0;
  gate.add_policy(std::make_shared<robot_safety::VelocityLimitPolicy>(limits));

  robot_safety::RobotCommand cmd;
  cmd.kind = robot_safety::CommandKind::Navigate;
  cmd.requested_linear_velocity = 0.5;
  cmd.requested_angular_velocity = 0.2;
  robot_safety::RobotState state;
  state.state_known = true;
  EXPECT_TRUE(gate.admit(cmd, state).allowed());
}
