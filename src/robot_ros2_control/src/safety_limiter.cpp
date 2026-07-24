#include "robot_ros2_control/safety_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace robot_ros2_control
{

SafetyLimiter::SafetyLimiter(HardwareParams params)
: params_(params)
{
}

double SafetyLimiter::clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

void SafetyLimiter::limit(CommandSnapshot & command) const
{
  // Intentionally no position clamp: controllers own the trajectory; NaN = leave unchanged.
  for (auto & velocity : command.velocity) {
    if (std::isfinite(velocity)) {
      velocity = clamp(velocity, -params_.max_velocity, params_.max_velocity);
    }
  }

  for (auto & effort : command.effort) {
    if (std::isfinite(effort)) {
      effort = clamp(effort, -params_.max_effort, params_.max_effort);
    }
  }
}

}  // namespace robot_ros2_control
