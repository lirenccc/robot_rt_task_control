#include "robot_hardware_plugins/mock_hardware_bus.hpp"

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>

namespace robot_hardware_plugins
{

bool MockHardwareBus::configure(
  const robot_hardware_api::HardwareParams & params,
  const std::vector<std::string> & joint_names,
  std::string & error)
{
  if (joint_names.empty()) {
    error = "MockHardwareBus requires at least one joint";
    return false;
  }

  params_ = params;
  joint_names_ = joint_names;
  plant_state_.resize(joint_names_.size());
  error.clear();
  return true;
}

bool MockHardwareBus::start(std::string & error)
{
  running_ = true;
  error.clear();
  return true;
}

void MockHardwareBus::stop()
{
  running_ = false;
}

bool MockHardwareBus::exchange(
  const robot_hardware_api::CommandSnapshot & command,
  robot_hardware_api::StateSnapshot & state,
  double dt_sec,
  std::string & error)
{
  if (!running_) {
    error = "MockHardwareBus is not running";
    return false;
  }

  if (command.position.size() != plant_state_.position.size()) {
    error = "Command size does not match plant state size";
    return false;
  }

  for (std::size_t i = 0; i < plant_state_.position.size(); ++i) {
    const double pos = plant_state_.position[i];
    const double vel = plant_state_.velocity[i];

    double desired_velocity = 0.0;

    if (std::isfinite(command.position[i])) {
      const double error_pos = command.position[i] - pos;
      desired_velocity += params_.position_kp * error_pos;
    }

    if (std::isfinite(command.velocity[i])) {
      desired_velocity += command.velocity[i];
    }

    desired_velocity -= params_.velocity_kd * vel;
    desired_velocity = std::clamp(desired_velocity, -params_.max_velocity, params_.max_velocity);

    plant_state_.velocity[i] = desired_velocity;
    plant_state_.position[i] += desired_velocity * dt_sec;
    plant_state_.effort[i] = std::isfinite(command.effort[i]) ? command.effort[i] : 0.0;
  }

  state = plant_state_;
  error.clear();
  return true;
}

}  // namespace robot_hardware_plugins

PLUGINLIB_EXPORT_CLASS(
  robot_hardware_plugins::MockHardwareBus,
  robot_hardware_api::HardwareBus)
