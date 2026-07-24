#ifdef ROBOT_HAS_NAV2

#include "robot_navigation_adapters/nav2_navigation_port.hpp"

#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

namespace robot_navigation_adapters
{

Nav2NavigationPort::Nav2NavigationPort(
  rclcpp::Node::SharedPtr node,
  std::string action_name)
: node_(std::move(node)),
  action_name_(std::move(action_name)),
  client_(rclcpp_action::create_client<Nav2NavigateToPose>(node_, action_name_))
{
}

robot_core_api::Result<robot_capability_api::GoalId> Nav2NavigationPort::start(
  const robot_capability_api::NavigateGoal & goal,
  robot_capability_api::NavigationFeedbackCallback feedback)
{
  using robot_core_api::ErrorCode;
  using robot_core_api::Result;

  if (!client_->wait_for_action_server(2s)) {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::Unavailable, "Nav2 action server unavailable", true);
  }

  Nav2NavigateToPose::Goal nav_goal;
  nav_goal.pose = goal.target_pose;
  // planner_id -> behavior_tree path is project-specific; leave empty for Nav2 default BT.
  (void)goal.planner_id;

  rclcpp_action::Client<Nav2NavigateToPose>::SendGoalOptions options;
  options.feedback_callback =
    [feedback](GoalHandle::SharedPtr, const std::shared_ptr<const Nav2NavigateToPose::Feedback> fb) {
      if (!feedback || !fb) {
        return;
      }
      robot_capability_api::NavigationFeedback out;
      out.phase = "nav2";
      out.distance_remaining = static_cast<float>(fb->distance_remaining);
      out.progress = out.distance_remaining > 0.01f
        ? std::max(0.0f, 1.0f - out.distance_remaining / 10.0f) : 1.0f;
      feedback(out);
    };

  auto goal_future = client_->async_send_goal(nav_goal, options);
  if (goal_future.wait_for(3s) != std::future_status::ready) {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::Timeout, "Nav2 goal request timed out", true);
  }
  auto handle = goal_future.get();
  if (!handle) {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::Unavailable, "Nav2 goal rejected");
  }

  const auto id = std::to_string(next_id_.fetch_add(1));
  ActiveGoal active;
  active.handle = handle;
  active.result_future = client_->async_get_result(handle);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goals_[id] = std::move(active);
  }
  return Result<robot_capability_api::GoalId>::success(id);
}

robot_core_api::Status Nav2NavigationPort::cancel(const robot_capability_api::GoalId & id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = goals_.find(id);
  if (it == goals_.end()) {
    return robot_core_api::Status::error(
      robot_core_api::ErrorCode::Unavailable, "Unknown Nav2 goal id");
  }
  client_->async_cancel_goal(it->second.handle);
  return robot_core_api::Status::success();
}

robot_core_api::Status Nav2NavigationPort::wait(
  const robot_capability_api::GoalId & id, double timeout_sec)
{
  ActiveGoal active;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = goals_.find(id);
    if (it == goals_.end()) {
      return robot_core_api::Status::error(
        robot_core_api::ErrorCode::Unavailable, "Unknown Nav2 goal id");
    }
    active = it->second;
  }

  if (active.result_future.wait_for(std::chrono::duration<double>(timeout_sec)) !=
    std::future_status::ready)
  {
    return robot_core_api::Status::error(
      robot_core_api::ErrorCode::Timeout, "Nav2 result timed out", true);
  }

  const auto wrapped = active.result_future.get();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goals_.erase(id);
  }

  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
    return robot_core_api::Status::success("Nav2 succeeded");
  }
  if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
    return robot_core_api::Status::error(robot_core_api::ErrorCode::Canceled, "Nav2 canceled");
  }
  return robot_core_api::Status::error(
    robot_core_api::ErrorCode::InternalError, "Nav2 failed");
}

robot_capability_api::CapabilityHealth Nav2NavigationPort::health() const
{
  robot_capability_api::CapabilityHealth h;
  h.available = client_ && client_->action_server_is_ready();
  h.detail = action_name_;
  return h;
}

}  // namespace robot_navigation_adapters

#endif  // ROBOT_HAS_NAV2
