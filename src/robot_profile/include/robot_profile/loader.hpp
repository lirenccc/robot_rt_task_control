#pragma once

#include <string>

#include "robot_core_api/result.hpp"
#include "robot_profile/robot_profile.hpp"

namespace robot_profile
{

class ProfileLoader
{
public:
  robot_core_api::Result<RobotProfile> load_file(const std::string & path) const;
  robot_core_api::Result<RobotProfile> load_string(const std::string & yaml_text) const;
};

}  // namespace robot_profile
