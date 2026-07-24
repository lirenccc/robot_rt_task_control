/**
 * @brief 模块 robot_task：任务请求、图规划与 TaskOrchestrator。
 *
 * 非实时编排导航/操作能力端口；不进入 HardwareBus 实时路径。
 */

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include "robot_capability_api/ports.hpp"
#include "robot_core_api/fault.hpp"
#include "robot_core_api/status.hpp"
#include "robot_safety/safety_gate.hpp"
#include "robot_task/planner.hpp"
#include "robot_task/task_types.hpp"

namespace robot_task
{

struct OrchestratorProgress
{
  std::string state;
  std::string step;
  float progress{0.0f};
};

using ProgressCallback = std::function<void(const OrchestratorProgress &)>;

class TaskOrchestrator
{
public:
  TaskOrchestrator(
    std::shared_ptr<robot_capability_api::INavigationPort> navigation,
    std::shared_ptr<robot_capability_api::IManipulationPort> manipulation,
    std::shared_ptr<robot_safety::SafetyGate> safety_gate,
    std::shared_ptr<robot_core_api::FaultManager> faults,
    std::shared_ptr<ITaskPlanner> planner);

  robot_core_api::Status execute(
    const TaskRequest & request,
    ProgressCallback progress,
    const std::atomic<bool> & cancel_requested);

private:
  std::shared_ptr<robot_capability_api::INavigationPort> navigation_;
  std::shared_ptr<robot_capability_api::IManipulationPort> manipulation_;
  std::shared_ptr<robot_safety::SafetyGate> safety_gate_;
  std::shared_ptr<robot_core_api::FaultManager> faults_;
  std::shared_ptr<ITaskPlanner> planner_;
};

}  // namespace robot_task
