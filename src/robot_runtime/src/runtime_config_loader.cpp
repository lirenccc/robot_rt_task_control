#include "robot_runtime/runtime_config_loader.hpp"

#include <fstream>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace robot_runtime
{
namespace
{

robot_core_api::Result<RuntimeConfig> parse_runtime(const YAML::Node & root)
{
  using robot_core_api::ErrorCode;
  using robot_core_api::Result;

  if (!root || !root.IsMap()) {
    return Result<RuntimeConfig>::failure(
      ErrorCode::InvalidConfiguration, "Runtime root must be a mapping");
  }

  RuntimeConfig cfg;
  try {
    if (!root["schema_version"]) {
      return Result<RuntimeConfig>::failure(
        ErrorCode::InvalidConfiguration, "Missing schema_version");
    }
    cfg.schema_version = root["schema_version"].as<int>();
    if (cfg.schema_version != 1) {
      return Result<RuntimeConfig>::failure(
        ErrorCode::InvalidConfiguration, "Unsupported runtime schema_version");
    }

    if (const auto hw = root["hardware"]) {
      if (const auto base = hw["base"]) {
        cfg.hardware_base.name = "base";
        cfg.hardware_base.plugin =
          base["plugin"] ? base["plugin"].as<std::string>() : "";
      }
    }
    if (cfg.hardware_base.plugin.empty()) {
      return Result<RuntimeConfig>::failure(
        ErrorCode::InvalidConfiguration, "hardware.base.plugin is required");
    }

    if (const auto providers = root["providers"]) {
      if (const auto nav = providers["navigation"]) {
        cfg.navigation.type = nav["type"] ? nav["type"].as<std::string>() : "ros_action";
        cfg.navigation.endpoint =
          nav["endpoint"] ? nav["endpoint"].as<std::string>() : "/navigation/navigate_to_pose";
      }
      if (const auto manip = providers["manipulation"]) {
        cfg.manipulation.type = manip["type"] ? manip["type"].as<std::string>() : "ros_action";
        cfg.manipulation.endpoint =
          manip["endpoint"] ? manip["endpoint"].as<std::string>() :
          "/manipulation/execute_skill";
      }
    }

    if (const auto safety = root["safety"]) {
      if (const auto policies = safety["policies"]) {
        for (const auto & p : policies) {
          cfg.safety_policies.push_back(p.as<std::string>());
        }
      }
    }

    if (const auto planner = root["planner"]) {
      cfg.planner.type = planner["type"] ? planner["type"].as<std::string>() : "simple";
      cfg.planner.tasks_dir =
        planner["tasks_dir"] ? planner["tasks_dir"].as<std::string>() : "";
    } else {
      cfg.planner.type = "simple";
    }
  } catch (const YAML::Exception & ex) {
    return Result<RuntimeConfig>::failure(
      ErrorCode::InvalidConfiguration, std::string("YAML parse error: ") + ex.what());
  }

  return Result<RuntimeConfig>::success(std::move(cfg));
}

}  // namespace

robot_core_api::Result<RuntimeConfig> RuntimeConfigLoader::load_file(const std::string & path) const
{
  std::ifstream input(path);
  if (!input) {
    return robot_core_api::Result<RuntimeConfig>::failure(
      robot_core_api::ErrorCode::InvalidConfiguration,
      "Cannot open runtime file: " + path);
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return load_string(buffer.str());
}

robot_core_api::Result<RuntimeConfig> RuntimeConfigLoader::load_string(
  const std::string & yaml_text) const
{
  try {
    return parse_runtime(YAML::Load(yaml_text));
  } catch (const YAML::Exception & ex) {
    return robot_core_api::Result<RuntimeConfig>::failure(
      robot_core_api::ErrorCode::InvalidConfiguration,
      std::string("YAML parse error: ") + ex.what());
  }
}

}  // namespace robot_runtime
