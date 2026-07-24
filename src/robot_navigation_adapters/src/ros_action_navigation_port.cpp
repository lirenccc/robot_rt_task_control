#include "robot_navigation_adapters/navigation_ports.hpp"

#include <chrono>
#include <future>

using namespace std::chrono_literals;

namespace robot_navigation_adapters
{

RosActionNavigationPort::RosActionNavigationPort(
  rclcpp::Node::SharedPtr node,
  std::string action_name)
: node_(std::move(node)),
  action_name_(std::move(action_name)),
  client_(rclcpp_action::create_client<NavigateToPose>(node_, action_name_))
{
}

robot_core_api::Result<robot_capability_api::GoalId> RosActionNavigationPort::start(
  const robot_capability_api::NavigateGoal & goal,
  robot_capability_api::NavigationFeedbackCallback feedback)
{
  using robot_core_api::ErrorCode;
  using robot_core_api::Result;

  if (!client_->wait_for_action_server(2s)) {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::Unavailable, "Navigation action server unavailable", true);
  }

  NavigateToPose::Goal ros_goal;
  ros_goal.target_pose = goal.target_pose;
  ros_goal.planner_id = goal.planner_id;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.feedback_callback =
    [feedback](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
      if (!feedback) {
        return;
      }
      robot_capability_api::NavigationFeedback out;
      out.phase = fb->phase;
      out.progress = fb->progress;
      out.distance_remaining = fb->distance_remaining;
      feedback(out);
    };

  auto goal_future = client_->async_send_goal(ros_goal, options);
  if (goal_future.wait_for(3s) != std::future_status::ready) {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::Timeout, "Navigation goal request timed out", true);
  }

  auto handle = goal_future.get();
  if (!handle) {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::Unavailable, "Navigation goal rejected");
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

robot_core_api::Status RosActionNavigationPort::cancel(const robot_capability_api::GoalId & id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = goals_.find(id);
  if (it == goals_.end()) {
    return robot_core_api::Status::error(
      robot_core_api::ErrorCode::Unavailable, "Unknown navigation goal id");
  }
  client_->async_cancel_goal(it->second.handle);
  return robot_core_api::Status::success();
}

robot_core_api::Status RosActionNavigationPort::wait(
  const robot_capability_api::GoalId & id, double timeout_sec)
{
  ActiveGoal active;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = goals_.find(id);
    if (it == goals_.end()) {
      return robot_core_api::Status::error(
        robot_core_api::ErrorCode::Unavailable, "Unknown navigation goal id");
    }
    active = it->second;
  }

  const auto timeout = std::chrono::duration<double>(timeout_sec);
  if (active.result_future.wait_for(timeout) != std::future_status::ready) {
    return robot_core_api::Status::error(
      robot_core_api::ErrorCode::Timeout, "Navigation result timed out", true);
  }

  const auto wrapped = active.result_future.get();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goals_.erase(id);
  }

  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
    wrapped.result && wrapped.result->success)
  {
    return robot_core_api::Status::success(wrapped.result->message);
  }
  if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
    return robot_core_api::Status::error(robot_core_api::ErrorCode::Canceled, "Navigation canceled");
  }
  return robot_core_api::Status::error(
    robot_core_api::ErrorCode::InternalError,
    wrapped.result ? wrapped.result->message : "Navigation failed");
}

robot_capability_api::CapabilityHealth RosActionNavigationPort::health() const
{
  robot_capability_api::CapabilityHealth h;
  h.available = client_ && client_->action_server_is_ready();
  h.detail = action_name_;
  return h;
}

}  // namespace robot_navigation_adapters
