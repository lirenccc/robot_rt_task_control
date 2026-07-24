#include "robot_safety/safety_gate.hpp"

namespace robot_safety
{

void SafetyGate::add_policy(std::shared_ptr<ISafetyPolicy> policy)
{
  if (policy) {
    policies_.push_back(std::move(policy));
  }
}

SafetyDecision SafetyGate::admit(const RobotCommand & command, const RobotState & state) const
{
  if (policies_.empty()) {
    return SafetyDecision::deny("No safety policies configured (default deny)");
  }
  if (!state.state_known) {
    return SafetyDecision::deny("Robot state unknown (default deny)");
  }
  if (state.estop_active) {
    return SafetyDecision::deny("E-stop active");
  }

  for (const auto & policy : policies_) {
    const auto decision = policy->admit(command, state);
    if (!decision.allowed()) {
      return decision;
    }
  }
  return SafetyDecision::allow();
}

SafetyDecision SafetyGate::monitor(const ActiveCommand & command, const RobotState & state) const
{
  if (policies_.empty()) {
    return SafetyDecision::abort_cmd("No safety policies configured (default deny)");
  }
  if (!state.state_known) {
    return SafetyDecision::abort_cmd("Robot state unknown during monitor");
  }
  if (state.estop_active) {
    return SafetyDecision::abort_cmd("E-stop active");
  }

  for (const auto & policy : policies_) {
    const auto decision = policy->monitor(command, state);
    if (decision.verdict != SafetyVerdict::Allow) {
      return decision;
    }
  }
  return SafetyDecision::allow();
}

}  // namespace robot_safety
