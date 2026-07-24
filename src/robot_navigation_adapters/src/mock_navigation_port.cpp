#include "robot_navigation_adapters/navigation_ports.hpp"

#include <chrono>
#include <thread>

namespace robot_navigation_adapters
{

robot_core_api::Result<robot_capability_api::GoalId> MockNavigationPort::start(
  const robot_capability_api::NavigateGoal &,
  robot_capability_api::NavigationFeedbackCallback feedback)
{
  const auto id = std::to_string(next_id_.fetch_add(1));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    canceled_[id] = false;
  }

  if (feedback) {
    for (int i = 0; i <= 5; ++i) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (canceled_[id]) {
          return robot_core_api::Result<robot_capability_api::GoalId>::failure(
            robot_core_api::ErrorCode::Canceled, "Navigation canceled");
        }
      }
      robot_capability_api::NavigationFeedback fb;
      fb.phase = i < 2 ? "global planning" : "tracking path";
      fb.progress = static_cast<float>(i) / 5.0f;
      fb.distance_remaining = static_cast<float>(5 - i) * 0.1f;
      feedback(fb);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  return robot_core_api::Result<robot_capability_api::GoalId>::success(id);
}

robot_core_api::Status MockNavigationPort::cancel(const robot_capability_api::GoalId & id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  canceled_[id] = true;
  return robot_core_api::Status::success();
}

robot_core_api::Status MockNavigationPort::wait(
  const robot_capability_api::GoalId & id, double)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (canceled_[id]) {
    return robot_core_api::Status::error(robot_core_api::ErrorCode::Canceled, "Navigation canceled");
  }
  canceled_.erase(id);
  return robot_core_api::Status::success("Mock navigation reached target");
}

robot_capability_api::CapabilityHealth MockNavigationPort::health() const
{
  return robot_capability_api::CapabilityHealth{true, "mock_navigation"};
}

}  // namespace robot_navigation_adapters
