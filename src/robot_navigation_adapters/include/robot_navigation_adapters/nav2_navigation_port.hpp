#pragma once

#ifdef ROBOT_HAS_NAV2

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "robot_capability_api/ports.hpp"

namespace robot_navigation_adapters
{

/// Generic Nav2 backend template: INavigationPort -> nav2_msgs/action/NavigateToPose.
class Nav2NavigationPort final : public robot_capability_api::INavigationPort
{
public:
  using Nav2NavigateToPose = nav2_msgs::action::NavigateToPose;

  explicit Nav2NavigationPort(
    rclcpp::Node::SharedPtr node,
    std::string action_name = "navigate_to_pose");

  robot_core_api::Result<robot_capability_api::GoalId> start(
    const robot_capability_api::NavigateGoal & goal,
    robot_capability_api::NavigationFeedbackCallback feedback) override;

  robot_core_api::Status cancel(const robot_capability_api::GoalId & id) override;
  robot_core_api::Status wait(const robot_capability_api::GoalId & id, double timeout_sec) override;
  robot_capability_api::CapabilityHealth health() const override;

private:
  using GoalHandle = rclcpp_action::ClientGoalHandle<Nav2NavigateToPose>;

  struct ActiveGoal
  {
    GoalHandle::SharedPtr handle;
    std::shared_future<GoalHandle::WrappedResult> result_future;
  };

  rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  rclcpp_action::Client<Nav2NavigateToPose>::SharedPtr client_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, ActiveGoal> goals_;
  std::atomic<uint64_t> next_id_{1};
};

}  // namespace robot_navigation_adapters

#endif  // ROBOT_HAS_NAV2
