#pragma once

#include "robot_profile/robot_profile.hpp"
#include "robot_safety/safety_policy.hpp"

namespace robot_safety
{

class VelocityLimitPolicy final : public ISafetyPolicy
{
public:
  VelocityLimitPolicy() = default;
  explicit VelocityLimitPolicy(robot_profile::Limits limits);

  void set_limits(robot_profile::Limits limits) { limits_ = std::move(limits); }

  SafetyDecision admit(const RobotCommand & command, const RobotState & state) const override;
  SafetyDecision monitor(const ActiveCommand & command, const RobotState & state) const override;

private:
  robot_profile::Limits limits_;
};

}  // namespace robot_safety
