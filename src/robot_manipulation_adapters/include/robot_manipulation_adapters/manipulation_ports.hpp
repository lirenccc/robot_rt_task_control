/**
 * @brief 模块 robot_manipulation_adapters：ManipulationPort 适配（轨迹 Action / Mock）。
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
#include "robot_interfaces/action/execute_manipulation.hpp"

namespace robot_manipulation_adapters
{

class RosActionManipulationPort final : public robot_capability_api::IManipulationPort
{
public:
  using ExecuteManipulation = robot_interfaces::action::ExecuteManipulation;

  explicit RosActionManipulationPort(
    rclcpp::Node::SharedPtr node,
    std::string action_name = "/manipulation/execute_skill");

  robot_core_api::Result<robot_capability_api::GoalId> execute(
    const robot_capability_api::ManipulationGoal & goal,
    robot_capability_api::ManipulationFeedbackCallback feedback) override;

  robot_core_api::Status cancel(const robot_capability_api::GoalId & id) override;
  robot_core_api::Status wait(const robot_capability_api::GoalId & id, double timeout_sec) override;
  robot_capability_api::CapabilityHealth health() const override;

private:
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteManipulation>;

  struct ActiveGoal
  {
    GoalHandle::SharedPtr handle;
    std::shared_future<GoalHandle::WrappedResult> result_future;
  };

  rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  rclcpp_action::Client<ExecuteManipulation>::SharedPtr client_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, ActiveGoal> goals_;
  std::atomic<uint64_t> next_id_{1};
};

class MockManipulationPort final : public robot_capability_api::IManipulationPort
{
public:
  robot_core_api::Result<robot_capability_api::GoalId> execute(
    const robot_capability_api::ManipulationGoal & goal,
    robot_capability_api::ManipulationFeedbackCallback feedback) override;

  robot_core_api::Status cancel(const robot_capability_api::GoalId & id) override;
  robot_core_api::Status wait(const robot_capability_api::GoalId & id, double timeout_sec) override;
  robot_capability_api::CapabilityHealth health() const override;

private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, bool> canceled_;
  std::atomic<uint64_t> next_id_{1};
};

}  // namespace robot_manipulation_adapters
