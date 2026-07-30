/**
 * @brief 模块 robot_profile：机型事实 Schema、YAML 加载与校验。
 *
 * 只描述机器人静态/半静态参数，不含运行时控制环。
 */

#pragma once

#include <string>
#include <vector>

namespace robot_profile
{

struct Capabilities
{
  bool navigation{false};
  bool manipulation{false};
  bool gripper{false};
};

struct Frames
{
  std::string base{"base_link"};
  std::string map{"map"};
  std::string tool{"tool0"};
};

struct Limits
{
  double max_linear_velocity{0.0};
  double max_angular_velocity{0.0};
  double payload_kg{0.0};
};

struct RobotIdentity
{
  std::string model;
  std::string variant;
  std::string serial_number;
};

/// Optional joint list for product bringup / controllers / URDF (schema v1 additive).
struct JointsSpec
{
  std::vector<std::string> names;
  std::string unit{"meter"};   // meter | radian
  std::string type{"revolute"};  // revolute | prismatic | mixed
  // Shared travel limits for generated RT URDF (SI: m or rad matching unit).
  double limit_lower{-0.06};
  double limit_upper{0.08};
  double max_effort{1000.0};
  double max_velocity{1.0};
};

/// Optional kinematics/dynamics plugin id (e.g. pm_6ucu). Empty → derive from robot.variant.
struct MechanismSpec
{
  std::string plugin;
};

/// Strongly-typed robot facts. Must not contain plugins for providers, Action endpoints, or Launch.
struct RobotProfile
{
  int schema_version{0};
  RobotIdentity robot;
  Capabilities capabilities;
  Frames frames;
  Limits limits;
  JointsSpec joints;
  MechanismSpec mechanism;

  /// Resolved mechanism plugin: mechanism.plugin, else robot.variant, else empty.
  std::string mechanism_plugin() const
  {
    if (!mechanism.plugin.empty()) {
      return mechanism.plugin;
    }
    return robot.variant;
  }
};

}  // namespace robot_profile
