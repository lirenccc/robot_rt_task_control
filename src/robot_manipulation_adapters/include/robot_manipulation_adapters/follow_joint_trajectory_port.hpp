#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "robot_capability_api/ports.hpp"

namespace robot_manipulation_adapters
{

/// Generic MoveIt / arm-controller backend template:
/// Generic trajectory backend: ManipulationGoal.joint_* -> FollowJointTrajectory.
/// Does not perform IK; callers / MoveIt must supply joint targets.
/// Does not embed a specific robot kinematic model — fill joint names via parameters.
class FollowJointTrajectoryManipulationPort final : public robot_capability_api::IManipulationPort
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  explicit FollowJointTrajectoryManipulationPort(
    rclcpp::Node::SharedPtr node,
    std::string action_name = "/arm_controller/follow_joint_trajectory",
    std::vector<std::string> joint_names = {
      "arm_joint_1", "arm_joint_2", "arm_joint_3", "gripper_joint"});

  robot_core_api::Result<robot_capability_api::GoalId> execute(
    const robot_capability_api::ManipulationGoal & goal,
    robot_capability_api::ManipulationFeedbackCallback feedback) override;

  robot_core_api::Status cancel(const robot_capability_api::GoalId & id) override;
  robot_core_api::Status wait(const robot_capability_api::GoalId & id, double timeout_sec) override;
  robot_capability_api::CapabilityHealth health() const override;

private:
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  struct ActiveGoal
  {
    GoalHandle::SharedPtr handle;
    std::shared_future<GoalHandle::WrappedResult> result_future;
  };

  rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  std::vector<std::string> joint_names_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, ActiveGoal> goals_;
  std::atomic<uint64_t> next_id_{1};
};

}  // namespace robot_manipulation_adapters
