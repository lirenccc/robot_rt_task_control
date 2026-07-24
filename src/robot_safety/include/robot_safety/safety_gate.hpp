/**
 * @brief 模块 robot_safety：SafetyPolicy 接口、组合 SafetyGate 与内置策略。
 *
 * 在任务/运动许可前做策略检查（如速度限制）；与主站 Job 内 safe-output 分层。
 */

#pragma once

#include <memory>
#include <vector>

#include "robot_safety/safety_policy.hpp"

namespace robot_safety
{

/// Composite gate. Default-deny: empty policy list or unknown state rejects commands.
class SafetyGate
{
public:
  void add_policy(std::shared_ptr<ISafetyPolicy> policy);

  SafetyDecision admit(const RobotCommand & command, const RobotState & state) const;
  SafetyDecision monitor(const ActiveCommand & command, const RobotState & state) const;

private:
  std::vector<std::shared_ptr<ISafetyPolicy>> policies_;
};

}  // namespace robot_safety
