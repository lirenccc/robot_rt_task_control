#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace robot_task
{

enum class TaskStepKind
{
  Navigate,
  Manipulate
};

struct TaskStep
{
  TaskStepKind kind;
  geometry_msgs::msg::PoseStamped pose;
  std::string skill_name;
  std::string object_id;
  std::string label;
  /// Optional explicit joint target for FollowJointTrajectory backends.
  std::vector<std::string> joint_names;
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;
  double duration_sec{1.0};
};

struct TaskRequest
{
  std::string instruction;
  std::string source{"action"};
};

/// Ordered task graph (MVP: sequential execution).
struct TaskGraph
{
  std::string instruction;
  std::vector<TaskStep> steps;
};

}  // namespace robot_task
