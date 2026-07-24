#include "robot_hardware_plugins/reference_sim_hardware_bus.hpp"

#include <cmath>

#include <pluginlib/class_list_macros.hpp>

namespace robot_hardware_plugins
{

bool ReferenceSimHardwareBus::configure(
  const robot_hardware_api::HardwareParams & params,
  const std::vector<std::string> & joint_names,
  std::string & error)
{
  if (joint_names.empty()) {
    error = "ReferenceSimHardwareBus requires at least one joint";
    return false;
  }
  params_ = params;
  joint_names_ = joint_names;
  last_state_.resize(joint_names_.size());
  inject_fault_ = params.sim_inject_fault;
  sequence_ = 0;
  time_since_command_sec_ = 0.0;
  error.clear();
  return true;
}

bool ReferenceSimHardwareBus::start(std::string & error)
{
  running_ = true;
  sequence_ = 0;
  time_since_command_sec_ = 0.0;
  error.clear();
  return true;
}

void ReferenceSimHardwareBus::stop()
{
  running_ = false;
}

bool ReferenceSimHardwareBus::exchange(
  const robot_hardware_api::CommandSnapshot & command,
  robot_hardware_api::StateSnapshot & state,
  double dt_sec,
  std::string & error)
{
  if (!running_) {
    error = "ReferenceSimHardwareBus is not running";
    return false;
  }
  if (inject_fault_) {
    error = "ReferenceSimHardwareBus injected fault";
    return false;
  }
  if (command.position.size() != last_state_.position.size()) {
    error = "Command size mismatch";
    return false;
  }

  bool any_finite = false;
  for (std::size_t i = 0; i < command.position.size(); ++i) {
    if (std::isfinite(command.position[i])) {
      any_finite = true;
      last_state_.position[i] = command.position[i];
    }
    if (std::isfinite(command.velocity[i])) {
      last_state_.velocity[i] = command.velocity[i];
    }
    if (std::isfinite(command.effort[i])) {
      last_state_.effort[i] = command.effort[i];
    }
  }

  if (any_finite) {
    time_since_command_sec_ = 0.0;
  } else {
    time_since_command_sec_ += dt_sec > 0.0 ? dt_sec : 0.0;
    if (params_.watchdog_timeout_sec > 0.0 &&
      time_since_command_sec_ > params_.watchdog_timeout_sec)
    {
      error = "ReferenceSimHardwareBus watchdog timeout";
      return false;
    }
  }

  ++sequence_;
  // Encode sequence into effort[0] LSB-style for tests without allocating.
  if (!last_state_.effort.empty()) {
    last_state_.effort[0] = static_cast<double>(sequence_ & 0xffffu);
  }

  state = last_state_;
  error.clear();
  return true;
}

}  // namespace robot_hardware_plugins

PLUGINLIB_EXPORT_CLASS(
  robot_hardware_plugins::ReferenceSimHardwareBus,
  robot_hardware_api::HardwareBus)
