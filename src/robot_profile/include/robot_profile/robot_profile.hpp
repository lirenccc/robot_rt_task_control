/**
 * @brief 模块 robot_profile：机型事实 Schema、YAML 加载与校验。
 *
 * 只描述机器人静态/半静态参数，不含运行时控制环。
 */

#pragma once

#include <string>

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

/// Strongly-typed robot facts. Must not contain plugins, Action endpoints, or Launch.
struct RobotProfile
{
  int schema_version{0};
  RobotIdentity robot;
  Capabilities capabilities;
  Frames frames;
  Limits limits;
};

}  // namespace robot_profile
