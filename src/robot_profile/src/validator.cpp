#include "robot_profile/validator.hpp"

namespace robot_profile
{

robot_core_api::Status ProfileValidator::validate(const RobotProfile & profile) const
{
  using robot_core_api::ErrorCode;
  using robot_core_api::Status;

  if (profile.schema_version != 1) {
    return Status::error(
      ErrorCode::InvalidConfiguration,
      "Unsupported schema_version (expected 1)");
  }
  if (profile.robot.model.empty()) {
    return Status::error(ErrorCode::InvalidConfiguration, "robot.model is required");
  }
  if (profile.frames.base.empty() || profile.frames.map.empty()) {
    return Status::error(ErrorCode::InvalidConfiguration, "frames.base and frames.map are required");
  }
  if (profile.limits.max_linear_velocity <= 0.0) {
    return Status::error(
      ErrorCode::InvalidConfiguration, "limits.max_linear_velocity must be > 0");
  }
  if (profile.limits.max_angular_velocity <= 0.0) {
    return Status::error(
      ErrorCode::InvalidConfiguration, "limits.max_angular_velocity must be > 0");
  }
  if (profile.limits.payload_kg < 0.0) {
    return Status::error(ErrorCode::InvalidConfiguration, "limits.payload_kg must be >= 0");
  }

  if (!profile.joints.names.empty()) {
    for (const auto & name : profile.joints.names) {
      if (name.empty()) {
        return Status::error(ErrorCode::InvalidConfiguration, "joints.names contains empty entry");
      }
    }
    if (profile.joints.unit != "meter" && profile.joints.unit != "radian") {
      return Status::error(
        ErrorCode::InvalidConfiguration, "joints.unit must be 'meter' or 'radian'");
    }
    if (profile.joints.type != "revolute" &&
      profile.joints.type != "prismatic" &&
      profile.joints.type != "mixed")
    {
      return Status::error(
        ErrorCode::InvalidConfiguration,
        "joints.type must be 'revolute', 'prismatic', or 'mixed'");
    }
    if (!(profile.joints.limit_lower < profile.joints.limit_upper)) {
      return Status::error(
        ErrorCode::InvalidConfiguration, "joints.limit_lower must be < limit_upper");
    }
    if (profile.joints.max_effort <= 0.0 || profile.joints.max_velocity <= 0.0) {
      return Status::error(
        ErrorCode::InvalidConfiguration,
        "joints.max_effort and joints.max_velocity must be > 0");
    }
  }

  return Status::success();
}

}  // namespace robot_profile
