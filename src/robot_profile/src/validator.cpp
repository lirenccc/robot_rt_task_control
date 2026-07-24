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
  return Status::success();
}

}  // namespace robot_profile
