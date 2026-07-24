#pragma once

#include "robot_core_api/result.hpp"
#include "robot_profile/robot_profile.hpp"
#include "robot_task/task_types.hpp"

namespace robot_task
{

class InstructionParser
{
public:
  robot_core_api::Result<TaskRequest> parse(const std::string & instruction) const;
};

/// Pluggable planner boundary (simple keyword planner, YAML graph, etc.).
class ITaskPlanner
{
public:
  virtual ~ITaskPlanner() = default;
  virtual robot_core_api::Result<TaskGraph> plan(const TaskRequest & request) const = 0;
};

/// Keyword / capability based planner (default).
class SimpleTaskPlanner final : public ITaskPlanner
{
public:
  explicit SimpleTaskPlanner(robot_profile::RobotProfile profile);

  robot_core_api::Result<TaskGraph> plan(const TaskRequest & request) const override;

private:
  robot_profile::RobotProfile profile_;
};

}  // namespace robot_task
