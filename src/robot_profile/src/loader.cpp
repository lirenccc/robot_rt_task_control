#include "robot_profile/loader.hpp"

#include <fstream>
#include <sstream>

#include <yaml-cpp/yaml.h>

#include "robot_core_api/status.hpp"
#include "robot_profile/validator.hpp"

namespace robot_profile
{
namespace
{

robot_core_api::Result<RobotProfile> parse_node(const YAML::Node & root)
{
  using robot_core_api::ErrorCode;
  using robot_core_api::Result;
  using robot_core_api::Status;

  if (!root || !root.IsMap()) {
    return Result<RobotProfile>::failure(
      ErrorCode::InvalidConfiguration, "Profile root must be a mapping");
  }

  RobotProfile profile;
  try {
    if (!root["schema_version"]) {
      return Result<RobotProfile>::failure(
        ErrorCode::InvalidConfiguration, "Missing schema_version");
    }
    profile.schema_version = root["schema_version"].as<int>();

    const auto robot = root["robot"];
    if (!robot) {
      return Result<RobotProfile>::failure(
        ErrorCode::InvalidConfiguration, "Missing robot section");
    }
    profile.robot.model = robot["model"] ? robot["model"].as<std::string>() : "";
    profile.robot.variant = robot["variant"] ? robot["variant"].as<std::string>() : "";
    profile.robot.serial_number =
      robot["serial_number"] ? robot["serial_number"].as<std::string>() : "";

    if (const auto caps = root["capabilities"]) {
      profile.capabilities.navigation =
        caps["navigation"] ? caps["navigation"].as<bool>() : false;
      profile.capabilities.manipulation =
        caps["manipulation"] ? caps["manipulation"].as<bool>() : false;
      profile.capabilities.gripper =
        caps["gripper"] ? caps["gripper"].as<bool>() : false;
    }

    if (const auto frames = root["frames"]) {
      if (frames["base"]) {
        profile.frames.base = frames["base"].as<std::string>();
      }
      if (frames["map"]) {
        profile.frames.map = frames["map"].as<std::string>();
      }
      if (frames["tool"]) {
        profile.frames.tool = frames["tool"].as<std::string>();
      }
    }

    if (const auto limits = root["limits"]) {
      if (limits["max_linear_velocity"]) {
        profile.limits.max_linear_velocity = limits["max_linear_velocity"].as<double>();
      }
      if (limits["max_angular_velocity"]) {
        profile.limits.max_angular_velocity = limits["max_angular_velocity"].as<double>();
      }
      if (limits["payload_kg"]) {
        profile.limits.payload_kg = limits["payload_kg"].as<double>();
      }
    }
  } catch (const YAML::Exception & ex) {
    return Result<RobotProfile>::failure(
      ErrorCode::InvalidConfiguration, std::string("YAML parse error: ") + ex.what());
  }

  ProfileValidator validator;
  const auto status = validator.validate(profile);
  if (!status.ok()) {
    return Result<RobotProfile>::failure(status);
  }
  return Result<RobotProfile>::success(std::move(profile));
}

}  // namespace

robot_core_api::Result<RobotProfile> ProfileLoader::load_file(const std::string & path) const
{
  std::ifstream input(path);
  if (!input) {
    return robot_core_api::Result<RobotProfile>::failure(
      robot_core_api::ErrorCode::InvalidConfiguration,
      "Cannot open profile file: " + path);
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return load_string(buffer.str());
}

robot_core_api::Result<RobotProfile> ProfileLoader::load_string(const std::string & yaml_text) const
{
  try {
    const YAML::Node root = YAML::Load(yaml_text);
    return parse_node(root);
  } catch (const YAML::Exception & ex) {
    return robot_core_api::Result<RobotProfile>::failure(
      robot_core_api::ErrorCode::InvalidConfiguration,
      std::string("YAML parse error: ") + ex.what());
  }
}

}  // namespace robot_profile
