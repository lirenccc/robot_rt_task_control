/**
 * @brief 模块 robot_navigation_adapters：NavigationPort 适配（ROS Action / 进程内 Mock）。
 */

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "robot_capability_api/ports.hpp"
#include "robot_interfaces/action/navigate_to_pose.hpp"

namespace robot_navigation_adapters
{

class RosActionNavigationPort final : public robot_capability_api::INavigationPort
{
public:
  using NavigateToPose = robot_interfaces::action::NavigateToPose;

  explicit RosActionNavigationPort(
    rclcpp::Node::SharedPtr node,
    std::string action_name = "/navigation/navigate_to_pose");

  robot_core_api::Result<robot_capability_api::GoalId> start(
    const robot_capability_api::NavigateGoal & goal,
    robot_capability_api::NavigationFeedbackCallback feedback) override;

  robot_core_api::Status cancel(const robot_capability_api::GoalId & id) override;
  robot_core_api::Status wait(const robot_capability_api::GoalId & id, double timeout_sec) override;
  robot_capability_api::CapabilityHealth health() const override;

private:
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  struct ActiveGoal
  {
    GoalHandle::SharedPtr handle;
    std::shared_future<GoalHandle::WrappedResult> result_future;
  };

  rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, ActiveGoal> goals_;
  std::atomic<uint64_t> next_id_{1};
};

/// In-process mock: same Port contract, no ROS Action required (BuiltinFactory / unit tests).
class MockNavigationPort final : public robot_capability_api::INavigationPort
{
public:
  robot_core_api::Result<robot_capability_api::GoalId> start(
    const robot_capability_api::NavigateGoal & goal,
    robot_capability_api::NavigationFeedbackCallback feedback) override;

  robot_core_api::Status cancel(const robot_capability_api::GoalId & id) override;
  robot_core_api::Status wait(const robot_capability_api::GoalId & id, double timeout_sec) override;
  robot_capability_api::CapabilityHealth health() const override;

private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, bool> canceled_;
  std::atomic<uint64_t> next_id_{1};
};

}  // namespace robot_navigation_adapters
