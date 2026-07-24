#include "robot_manipulation_adapters/manipulation_ports.hpp"

#include <chrono>
#include <future>

using namespace std::chrono_literals;

namespace robot_manipulation_adapters
{

RosActionManipulationPort::RosActionManipulationPort(
  rclcpp::Node::SharedPtr node,
  std::string action_name)
: node_(std::move(node)),
  action_name_(std::move(action_name)),
  client_(rclcpp_action::create_client<ExecuteManipulation>(node_, action_name_))
{
}

robot_core_api::Result<robot_capability_api::GoalId> RosActionManipulationPort::execute(
  const robot_capability_api::ManipulationGoal & goal,
  robot_capability_api::ManipulationFeedbackCallback feedback)
{
  using robot_core_api::ErrorCode;
  using robot_core_api::Result;

  if (!client_->wait_for_action_server(2s)) {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::Unavailable, "Manipulation action server unavailable", true);
  }

  ExecuteManipulation::Goal ros_goal;
  ros_goal.skill_name = goal.skill_name;
  ros_goal.object_id = goal.object_id;
  ros_goal.target_pose = goal.target_pose;
  ros_goal.joint_names = goal.joint_names;
  ros_goal.joint_positions = goal.joint_positions;
  ros_goal.joint_velocities = goal.joint_velocities;
  ros_goal.duration_sec = goal.duration_sec;

  rclcpp_action::Client<ExecuteManipulation>::SendGoalOptions options;
  options.feedback_callback =
    [feedback](GoalHandle::SharedPtr, const std::shared_ptr<const ExecuteManipulation::Feedback> fb) {
      if (!feedback) {
        return;
      }
      robot_capability_api::ManipulationFeedback out;
      out.phase = fb->phase;
      out.progress = fb->progress;
      feedback(out);
    };

  auto goal_future = client_->async_send_goal(ros_goal, options);
  if (goal_future.wait_for(3s) != std::future_status::ready) {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::Timeout, "Manipulation goal request timed out", true);
  }

  auto handle = goal_future.get();
  if (!handle) {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::Unavailable, "Manipulation goal rejected");
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

robot_core_api::Status RosActionManipulationPort::cancel(const robot_capability_api::GoalId & id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = goals_.find(id);
  if (it == goals_.end()) {
    return robot_core_api::Status::error(
      robot_core_api::ErrorCode::Unavailable, "Unknown manipulation goal id");
  }
  client_->async_cancel_goal(it->second.handle);
  return robot_core_api::Status::success();
}

robot_core_api::Status RosActionManipulationPort::wait(
  const robot_capability_api::GoalId & id, double timeout_sec)
{
  ActiveGoal active;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = goals_.find(id);
    if (it == goals_.end()) {
      return robot_core_api::Status::error(
        robot_core_api::ErrorCode::Unavailable, "Unknown manipulation goal id");
    }
    active = it->second;
  }

  if (active.result_future.wait_for(std::chrono::duration<double>(timeout_sec)) !=
    std::future_status::ready)
  {
    return robot_core_api::Status::error(
      robot_core_api::ErrorCode::Timeout, "Manipulation result timed out", true);
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
    return robot_core_api::Status::error(
      robot_core_api::ErrorCode::Canceled, "Manipulation canceled");
  }
  return robot_core_api::Status::error(
    robot_core_api::ErrorCode::InternalError,
    wrapped.result ? wrapped.result->message : "Manipulation failed");
}

robot_capability_api::CapabilityHealth RosActionManipulationPort::health() const
{
  robot_capability_api::CapabilityHealth h;
  h.available = client_ && client_->action_server_is_ready();
  h.detail = action_name_;
  return h;
}

}  // namespace robot_manipulation_adapters
