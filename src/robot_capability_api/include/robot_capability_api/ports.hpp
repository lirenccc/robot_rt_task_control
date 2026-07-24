/**
 * @brief 模块 robot_capability_api：导航与操作能力端口抽象。
 *
 * TaskOrchestrator 通过端口调用适配器，不直接依赖 Nav2 / MoveIt 细节。
 */

#pragma once

#include <functional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "robot_core_api/result.hpp"
#include "robot_core_api/status.hpp"

namespace robot_capability_api
{

using GoalId = std::string;

struct CapabilityHealth
{
  bool available{false};
  std::string detail;
};

struct NavigateGoal
{
  geometry_msgs::msg::PoseStamped target_pose;
  std::string planner_id{"default"};
};

struct NavigationFeedback
{
  std::string phase;
  float progress{0.0f};
  float distance_remaining{0.0f};
};

using NavigationFeedbackCallback = std::function<void(const NavigationFeedback &)>;

class INavigationPort
{
public:
  virtual ~INavigationPort() = default;

  virtual robot_core_api::Result<GoalId> start(
    const NavigateGoal & goal,
    NavigationFeedbackCallback feedback) = 0;

  virtual robot_core_api::Status cancel(const GoalId & id) = 0;
  virtual robot_core_api::Status wait(const GoalId & id, double timeout_sec) = 0;
  virtual CapabilityHealth health() const = 0;
};

struct ManipulationGoal
{
  std::string skill_name;
  std::string object_id;
  geometry_msgs::msg::PoseStamped target_pose;
  /// Explicit joint target for trajectory backends. Empty positions => no joint command.
  std::vector<std::string> joint_names;
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;
  double duration_sec{1.0};

  bool has_joint_target() const { return !joint_positions.empty(); }
};

struct ManipulationFeedback
{
  std::string phase;
  float progress{0.0f};
};

using ManipulationFeedbackCallback = std::function<void(const ManipulationFeedback &)>;

class IManipulationPort
{
public:
  virtual ~IManipulationPort() = default;

  virtual robot_core_api::Result<GoalId> execute(
    const ManipulationGoal & goal,
    ManipulationFeedbackCallback feedback) = 0;

  virtual robot_core_api::Status cancel(const GoalId & id) = 0;
  virtual robot_core_api::Status wait(const GoalId & id, double timeout_sec) = 0;
  virtual CapabilityHealth health() const = 0;
};

}  // namespace robot_capability_api
