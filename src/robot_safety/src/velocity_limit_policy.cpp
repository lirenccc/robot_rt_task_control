#include "robot_safety/velocity_limit_policy.hpp"

#include <cmath>

#include <pluginlib/class_list_macros.hpp>

namespace robot_safety
{

VelocityLimitPolicy::VelocityLimitPolicy(robot_profile::Limits limits)
: limits_(limits)
{
}

SafetyDecision VelocityLimitPolicy::admit(
  const RobotCommand & command,
  const RobotState &) const
{
  if (limits_.max_linear_velocity <= 0.0 || limits_.max_angular_velocity <= 0.0) {
    return SafetyDecision::deny("Invalid velocity limits in profile");
  }
  if (std::abs(command.requested_linear_velocity) > limits_.max_linear_velocity) {
    return SafetyDecision::deny("Requested linear velocity exceeds profile limit");
  }
  if (std::abs(command.requested_angular_velocity) > limits_.max_angular_velocity) {
    return SafetyDecision::deny("Requested angular velocity exceeds profile limit");
  }
  return SafetyDecision::allow();
}

SafetyDecision VelocityLimitPolicy::monitor(
  const ActiveCommand & command,
  const RobotState & state) const
{
  if (std::abs(state.linear_velocity) > limits_.max_linear_velocity * 1.1) {
    return SafetyDecision::abort_cmd("Measured linear velocity exceeds limit");
  }
  if (std::abs(state.angular_velocity) > limits_.max_angular_velocity * 1.1) {
    return SafetyDecision::abort_cmd("Measured angular velocity exceeds limit");
  }
  (void)command;
  return SafetyDecision::allow();
}

}  // namespace robot_safety

PLUGINLIB_EXPORT_CLASS(robot_safety::VelocityLimitPolicy, robot_safety::ISafetyPolicy)
