#include "robot_manipulation_adapters/follow_joint_trajectory_port.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace robot_manipulation_adapters
{

FollowJointTrajectoryManipulationPort::FollowJointTrajectoryManipulationPort(
  rclcpp::Node::SharedPtr node,
  std::string action_name,
  std::vector<std::string> joint_names)
: node_(std::move(node)),
  action_name_(std::move(action_name)),
  joint_names_(std::move(joint_names)),
  client_(rclcpp_action::create_client<FollowJointTrajectory>(node_, action_name_))
{
}

robot_core_api::Result<robot_capability_api::GoalId>
FollowJointTrajectoryManipulationPort::execute(
  const robot_capability_api::ManipulationGoal & goal,
  robot_capability_api::ManipulationFeedbackCallback feedback)
{
  using robot_core_api::ErrorCode;
  using robot_core_api::Result;

  if (!goal.has_joint_target()) {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::InvalidArgument,
      "FollowJointTrajectory requires ManipulationGoal.joint_positions "
      "(no IK in this adapter; plan joints externally / via MoveIt)");
  }

  const std::vector<std::string> & names =
    goal.joint_names.empty() ? joint_names_ : goal.joint_names;
  if (names.empty()) {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::InvalidConfiguration,
      "No joint_names on goal or port defaults");
  }
  if (goal.joint_positions.size() != names.size()) {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::InvalidArgument,
      "joint_positions size must match joint_names");
  }
  if (!goal.joint_velocities.empty() &&
    goal.joint_velocities.size() != names.size())
  {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::InvalidArgument,
      "joint_velocities size must match joint_names when provided");
  }

  if (!client_->wait_for_action_server(2s)) {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::Unavailable,
      "FollowJointTrajectory server unavailable (MoveIt/controller not up)", true);
  }

  FollowJointTrajectory::Goal traj_goal;
  traj_goal.trajectory.joint_names = names;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = goal.joint_positions;
  if (!goal.joint_velocities.empty()) {
    point.velocities = goal.joint_velocities;
  }
  const double duration = goal.duration_sec > 0.0 ? goal.duration_sec : 1.0;
  point.time_from_start = rclcpp::Duration::from_seconds(duration);
  traj_goal.trajectory.points.push_back(point);

  if (feedback) {
    robot_capability_api::ManipulationFeedback fb;
    fb.phase = "follow_joint_trajectory:" + goal.skill_name;
    fb.progress = 0.1f;
    feedback(fb);
  }

  auto goal_future = client_->async_send_goal(traj_goal);
  if (goal_future.wait_for(3s) != std::future_status::ready) {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::Timeout, "Trajectory goal request timed out", true);
  }
  auto handle = goal_future.get();
  if (!handle) {
    return Result<robot_capability_api::GoalId>::failure(
      ErrorCode::Unavailable, "Trajectory goal rejected");
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

robot_core_api::Status FollowJointTrajectoryManipulationPort::cancel(
  const robot_capability_api::GoalId & id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = goals_.find(id);
  if (it == goals_.end()) {
    return robot_core_api::Status::error(
      robot_core_api::ErrorCode::Unavailable, "Unknown trajectory goal id");
  }
  client_->async_cancel_goal(it->second.handle);
  return robot_core_api::Status::success();
}

robot_core_api::Status FollowJointTrajectoryManipulationPort::wait(
  const robot_capability_api::GoalId & id, double timeout_sec)
{
  ActiveGoal active;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = goals_.find(id);
    if (it == goals_.end()) {
      return robot_core_api::Status::error(
        robot_core_api::ErrorCode::Unavailable, "Unknown trajectory goal id");
    }
    active = it->second;
  }

  if (active.result_future.wait_for(std::chrono::duration<double>(timeout_sec)) !=
    std::future_status::ready)
  {
    return robot_core_api::Status::error(
      robot_core_api::ErrorCode::Timeout, "Trajectory result timed out", true);
  }

  const auto wrapped = active.result_future.get();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goals_.erase(id);
  }

  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
    return robot_core_api::Status::success("FollowJointTrajectory succeeded");
  }
  if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
    return robot_core_api::Status::error(
      robot_core_api::ErrorCode::Canceled, "FollowJointTrajectory canceled");
  }
  return robot_core_api::Status::error(
    robot_core_api::ErrorCode::InternalError, "FollowJointTrajectory failed");
}

robot_capability_api::CapabilityHealth
FollowJointTrajectoryManipulationPort::health() const
{
  robot_capability_api::CapabilityHealth h;
  h.available = client_ && client_->action_server_is_ready();
  h.detail = action_name_;
  return h;
}

}  // namespace robot_manipulation_adapters
