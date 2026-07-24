#include "robot_hardware_plugins/skeleton_hardware_bus.hpp"

#include <cmath>

#include <pluginlib/class_list_macros.hpp>

namespace robot_hardware_plugins
{

bool SkeletonHardwareBus::configure(
  const robot_hardware_api::HardwareParams & params,
  const std::vector<std::string> & joint_names,
  std::string & error)
{
  if (joint_names.empty()) {
    error = "SkeletonHardwareBus requires at least one joint";
    return false;
  }

  // TODO(vendor): open transport (EtherCAT master, CAN socket, serial, etc.)
  // TODO(vendor): build joint_name -> device id map
  // TODO(vendor): validate PDO / register layout against joint_names.size()

  params_ = params;
  joint_names_ = joint_names;
  last_state_.resize(joint_names_.size());
  transport_ready_ = true;  // template: pretend transport opened
  error.clear();
  return true;
}

bool SkeletonHardwareBus::start(std::string & error)
{
  if (!transport_ready_) {
    error = "SkeletonHardwareBus not configured";
    return false;
  }

  // TODO(vendor): enable drives / clear faults / arm watchdog
  running_ = true;
  error.clear();
  return true;
}

void SkeletonHardwareBus::stop()
{
  // TODO(vendor): disable drives, close transport if owned here
  running_ = false;
}

bool SkeletonHardwareBus::exchange(
  const robot_hardware_api::CommandSnapshot & command,
  robot_hardware_api::StateSnapshot & state,
  double /*dt_sec*/,
  std::string & error)
{
  if (!running_) {
    error = "SkeletonHardwareBus is not running";
    return false;
  }

  if (command.position.size() != last_state_.position.size()) {
    error = "Command size mismatch";
    return false;
  }

  // TODO(vendor): pack command into bus frames (no heap allocation)
  // TODO(vendor): cycle the bus, unpack state with CRC / sequence checks
  // Template behavior: hold last state (safe no-op plant) so the RT loop can run.
  for (std::size_t i = 0; i < last_state_.position.size(); ++i) {
    if (std::isfinite(command.position[i])) {
      // Placeholder: echo commanded position as measured (replace with encoder readback)
      last_state_.position[i] = command.position[i];
    }
    if (std::isfinite(command.velocity[i])) {
      last_state_.velocity[i] = command.velocity[i];
    }
    if (std::isfinite(command.effort[i])) {
      last_state_.effort[i] = command.effort[i];
    }
  }

  state = last_state_;
  error.clear();
  return true;
}

}  // namespace robot_hardware_plugins

PLUGINLIB_EXPORT_CLASS(
  robot_hardware_plugins::SkeletonHardwareBus,
  robot_hardware_api::HardwareBus)
