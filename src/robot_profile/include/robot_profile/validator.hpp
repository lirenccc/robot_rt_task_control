#pragma once

#include "robot_core_api/status.hpp"
#include "robot_profile/robot_profile.hpp"

namespace robot_profile
{

class ProfileValidator
{
public:
  robot_core_api::Status validate(const RobotProfile & profile) const;
};

}  // namespace robot_profile
