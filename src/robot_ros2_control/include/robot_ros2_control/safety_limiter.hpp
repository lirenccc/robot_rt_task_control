#pragma once

#include <cstddef>

#include "robot_ros2_control/types.hpp"

namespace robot_ros2_control
{

/// Soft clamps applied on the RT path before HardwareBus::exchange.
/// Limits velocity and effort only: position is left to the controller / bus hold logic
/// (NaN means "no setpoint"). Non-finite values are skipped so quiet_nan position markers
/// and unset channels pass through. position_kp is not used here (plant buses only).
class SafetyLimiter
{
public:
  explicit SafetyLimiter(HardwareParams params);

  void limit(CommandSnapshot & command) const;

private:
  static double clamp(double value, double min_value, double max_value);

  HardwareParams params_;
};

}  // namespace robot_ros2_control
