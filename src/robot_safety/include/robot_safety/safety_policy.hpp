#pragma once

#include <string>

#include "robot_core_api/status.hpp"

namespace robot_safety
{

enum class SafetyVerdict
{
  Allow,
  Deny,
  Abort
};

struct SafetyDecision
{
  SafetyVerdict verdict{SafetyVerdict::Deny};
  std::string reason;

  static SafetyDecision allow(std::string reason = {})
  {
    return SafetyDecision{SafetyVerdict::Allow, std::move(reason)};
  }

  static SafetyDecision deny(std::string reason)
  {
    return SafetyDecision{SafetyVerdict::Deny, std::move(reason)};
  }

  static SafetyDecision abort_cmd(std::string reason)
  {
    return SafetyDecision{SafetyVerdict::Abort, std::move(reason)};
  }

  bool allowed() const { return verdict == SafetyVerdict::Allow; }
};

enum class CommandKind
{
  Navigate,
  Manipulate,
  Unknown
};

struct RobotCommand
{
  CommandKind kind{CommandKind::Unknown};
  double requested_linear_velocity{0.0};
  double requested_angular_velocity{0.0};
  std::string label;
};

struct RobotState
{
  bool state_known{false};
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  bool estop_active{false};
};

struct ActiveCommand
{
  RobotCommand command;
  std::string goal_id;
};

class ISafetyPolicy
{
public:
  virtual ~ISafetyPolicy() = default;

  virtual SafetyDecision admit(
    const RobotCommand & command,
    const RobotState & state) const = 0;

  virtual SafetyDecision monitor(
    const ActiveCommand & command,
    const RobotState & state) const = 0;
};

}  // namespace robot_safety
