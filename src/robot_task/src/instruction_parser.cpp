#include "robot_task/planner.hpp"

#include <algorithm>
#include <cctype>

namespace robot_task
{
namespace
{

bool contains(std::string text, const std::string & needle)
{
  std::transform(text.begin(), text.end(), text.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text.find(needle) != std::string::npos;
}

}  // namespace

robot_core_api::Result<TaskRequest> InstructionParser::parse(const std::string & instruction) const
{
  if (instruction.empty()) {
    return robot_core_api::Result<TaskRequest>::failure(
      robot_core_api::ErrorCode::InvalidConfiguration, "Empty instruction");
  }
  TaskRequest request;
  request.instruction = instruction;
  return robot_core_api::Result<TaskRequest>::success(std::move(request));
}

SimpleTaskPlanner::SimpleTaskPlanner(robot_profile::RobotProfile profile)
: profile_(std::move(profile))
{
}

robot_core_api::Result<TaskGraph> SimpleTaskPlanner::plan(const TaskRequest & request) const
{
  const auto & instruction = request.instruction;

  const bool needs_manipulation = contains(instruction, "pick") ||
    contains(instruction, "grasp") ||
    contains(instruction, "place") ||
    contains(instruction, "抓") ||
    contains(instruction, "拿") ||
    contains(instruction, "放");

  const bool needs_navigation = contains(instruction, "go") ||
    contains(instruction, "navigate") ||
    contains(instruction, "table") ||
    contains(instruction, "到") ||
    contains(instruction, "移动") ||
    needs_manipulation;

  if (needs_navigation && !profile_.capabilities.navigation) {
    return robot_core_api::Result<TaskGraph>::failure(
      robot_core_api::ErrorCode::Unavailable, "Profile lacks navigation capability");
  }
  if (needs_manipulation && !profile_.capabilities.manipulation) {
    return robot_core_api::Result<TaskGraph>::failure(
      robot_core_api::ErrorCode::Unavailable, "Profile lacks manipulation capability");
  }

  TaskGraph graph;
  graph.instruction = instruction;

  if (needs_navigation) {
    TaskStep step;
    step.kind = TaskStepKind::Navigate;
    step.label = "navigate";
    step.pose.header.frame_id = profile_.frames.map;
    step.pose.pose.position.x = 1.0;
    step.pose.pose.orientation.w = 1.0;
    graph.steps.push_back(step);
  }

  if (needs_manipulation) {
    TaskStep step;
    step.kind = TaskStepKind::Manipulate;
    step.label = "manipulate";
    step.skill_name = contains(instruction, "place") || contains(instruction, "放")
      ? "pick_and_place" : "pick";
    step.object_id = "target_object";
    step.pose.header.frame_id = profile_.frames.base;
    step.pose.pose.position.x = 0.45;
    step.pose.pose.position.z = 0.25;
    step.pose.pose.orientation.w = 1.0;
    graph.steps.push_back(step);
  }

  if (graph.steps.empty()) {
    return robot_core_api::Result<TaskGraph>::failure(
      robot_core_api::ErrorCode::InvalidConfiguration,
      "Instruction did not map to any task steps");
  }

  return robot_core_api::Result<TaskGraph>::success(std::move(graph));
}

}  // namespace robot_task
