#pragma once

#include <string>

#include "robot_core_api/result.hpp"
#include "robot_profile/robot_profile.hpp"
#include "robot_task/planner.hpp"

namespace robot_task
{

/// Loads sequential task graphs from YAML files under a tasks directory.
/// Instruction forms: "graph:<name>" or bare "<name>" when only graph planner is selected.
class YamlGraphPlanner final : public ITaskPlanner
{
public:
  YamlGraphPlanner(robot_profile::RobotProfile profile, std::string tasks_dir);

  robot_core_api::Result<TaskGraph> plan(const TaskRequest & request) const override;

  static robot_core_api::Result<TaskGraph> load_file(
    const std::string & path,
    const robot_profile::RobotProfile & profile);

private:
  robot_profile::RobotProfile profile_;
  std::string tasks_dir_;
};

}  // namespace robot_task
