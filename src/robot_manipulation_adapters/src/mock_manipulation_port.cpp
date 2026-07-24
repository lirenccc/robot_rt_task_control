#include "robot_manipulation_adapters/manipulation_ports.hpp"

#include <chrono>
#include <thread>
#include <vector>

namespace robot_manipulation_adapters
{

robot_core_api::Result<robot_capability_api::GoalId> MockManipulationPort::execute(
  const robot_capability_api::ManipulationGoal &,
  robot_capability_api::ManipulationFeedbackCallback feedback)
{
  const auto id = std::to_string(next_id_.fetch_add(1));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    canceled_[id] = false;
  }

  const std::vector<std::string> phases{
    "detect object", "plan pregrasp", "execute approach", "close gripper", "lift object", "verify"};

  if (feedback) {
    for (std::size_t i = 0; i < phases.size(); ++i) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (canceled_[id]) {
          return robot_core_api::Result<robot_capability_api::GoalId>::failure(
            robot_core_api::ErrorCode::Canceled, "Manipulation canceled");
        }
      }
      robot_capability_api::ManipulationFeedback fb;
      fb.phase = phases[i];
      fb.progress = static_cast<float>(i + 1) / static_cast<float>(phases.size());
      feedback(fb);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  return robot_core_api::Result<robot_capability_api::GoalId>::success(id);
}

robot_core_api::Status MockManipulationPort::cancel(const robot_capability_api::GoalId & id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  canceled_[id] = true;
  return robot_core_api::Status::success();
}

robot_core_api::Status MockManipulationPort::wait(
  const robot_capability_api::GoalId & id, double)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (canceled_[id]) {
    return robot_core_api::Status::error(
      robot_core_api::ErrorCode::Canceled, "Manipulation canceled");
  }
  canceled_.erase(id);
  return robot_core_api::Status::success("Mock manipulation complete");
}

robot_capability_api::CapabilityHealth MockManipulationPort::health() const
{
  return robot_capability_api::CapabilityHealth{true, "mock_manipulation"};
}

}  // namespace robot_manipulation_adapters
