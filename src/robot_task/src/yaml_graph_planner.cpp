#include "robot_task/yaml_graph_planner.hpp"

#include <fstream>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace robot_task
{
namespace
{

geometry_msgs::msg::PoseStamped pose_from_yaml(
  const YAML::Node & node,
  const std::string & default_frame)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = node["frame_id"] ? node["frame_id"].as<std::string>() : default_frame;
  pose.pose.orientation.w = 1.0;
  if (const auto p = node["position"]) {
    pose.pose.position.x = p["x"] ? p["x"].as<double>() : 0.0;
    pose.pose.position.y = p["y"] ? p["y"].as<double>() : 0.0;
    pose.pose.position.z = p["z"] ? p["z"].as<double>() : 0.0;
  }
  if (const auto o = node["orientation"]) {
    pose.pose.orientation.x = o["x"] ? o["x"].as<double>() : 0.0;
    pose.pose.orientation.y = o["y"] ? o["y"].as<double>() : 0.0;
    pose.pose.orientation.z = o["z"] ? o["z"].as<double>() : 0.0;
    pose.pose.orientation.w = o["w"] ? o["w"].as<double>() : 1.0;
  }
  return pose;
}

}  // namespace

YamlGraphPlanner::YamlGraphPlanner(robot_profile::RobotProfile profile, std::string tasks_dir)
: profile_(std::move(profile)),
  tasks_dir_(std::move(tasks_dir))
{
}

robot_core_api::Result<TaskGraph> YamlGraphPlanner::plan(const TaskRequest & request) const
{
  std::string name = request.instruction;
  constexpr const char * kPrefix = "graph:";
  if (name.rfind(kPrefix, 0) == 0) {
    name = name.substr(std::char_traits<char>::length(kPrefix));
  }
  if (name.empty()) {
    return robot_core_api::Result<TaskGraph>::failure(
      robot_core_api::ErrorCode::InvalidArgument, "Empty graph name");
  }
  const std::string path = tasks_dir_ + "/" + name + ".yaml";
  return load_file(path, profile_);
}

robot_core_api::Result<TaskGraph> YamlGraphPlanner::load_file(
  const std::string & path,
  const robot_profile::RobotProfile & profile)
{
  using robot_core_api::ErrorCode;
  using robot_core_api::Result;

  std::ifstream input(path);
  if (!input) {
    return Result<TaskGraph>::failure(
      ErrorCode::InvalidConfiguration, "Cannot open task graph: " + path);
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();

  try {
    const YAML::Node root = YAML::Load(buffer.str());
    if (!root || !root.IsMap()) {
      return Result<TaskGraph>::failure(
        ErrorCode::InvalidConfiguration, "Task graph root must be a mapping");
    }
    if (!root["steps"] || !root["steps"].IsSequence() || root["steps"].size() == 0) {
      return Result<TaskGraph>::failure(
        ErrorCode::InvalidConfiguration, "Task graph requires non-empty steps");
    }

    TaskGraph graph;
    graph.instruction = root["name"] ? root["name"].as<std::string>() : path;

    for (const auto & step_node : root["steps"]) {
      TaskStep step;
      const std::string kind = step_node["kind"] ? step_node["kind"].as<std::string>() : "";
      step.label = step_node["label"] ? step_node["label"].as<std::string>() : kind;
      if (kind == "navigate") {
        if (!profile.capabilities.navigation) {
          return Result<TaskGraph>::failure(
            ErrorCode::Unavailable, "Profile lacks navigation capability");
        }
        step.kind = TaskStepKind::Navigate;
        step.pose = pose_from_yaml(
          step_node["pose"] ? step_node["pose"] : YAML::Node(YAML::NodeType::Map),
          profile.frames.map);
      } else if (kind == "manipulate") {
        if (!profile.capabilities.manipulation) {
          return Result<TaskGraph>::failure(
            ErrorCode::Unavailable, "Profile lacks manipulation capability");
        }
        step.kind = TaskStepKind::Manipulate;
        step.skill_name =
          step_node["skill_name"] ? step_node["skill_name"].as<std::string>() : "pick";
        step.object_id =
          step_node["object_id"] ? step_node["object_id"].as<std::string>() : "";
        step.pose = pose_from_yaml(
          step_node["pose"] ? step_node["pose"] : YAML::Node(YAML::NodeType::Map),
          profile.frames.base);
        if (const auto jn = step_node["joint_names"]) {
          for (const auto & n : jn) {
            step.joint_names.push_back(n.as<std::string>());
          }
        }
        if (const auto jp = step_node["joint_positions"]) {
          for (const auto & v : jp) {
            step.joint_positions.push_back(v.as<double>());
          }
        }
        if (const auto jv = step_node["joint_velocities"]) {
          for (const auto & v : jv) {
            step.joint_velocities.push_back(v.as<double>());
          }
        }
        if (step_node["duration_sec"]) {
          step.duration_sec = step_node["duration_sec"].as<double>();
        }
      } else {
        return Result<TaskGraph>::failure(
          ErrorCode::InvalidConfiguration, "Unknown step kind: " + kind);
      }
      graph.steps.push_back(std::move(step));
    }
    return Result<TaskGraph>::success(std::move(graph));
  } catch (const YAML::Exception & ex) {
    return Result<TaskGraph>::failure(
      ErrorCode::InvalidConfiguration, std::string("Task graph YAML error: ") + ex.what());
  }
}

}  // namespace robot_task
