#include "robot_ethercat_adapters/igh_hardware_bus.hpp"

#include <cmath>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

#include "robot_ethercat_adapters/axis_mapping.hpp"

namespace robot_ethercat_adapters
{

IghHardwareBus::~IghHardwareBus()
{
  stop();
}

bool IghHardwareBus::configure(
  const robot_hardware_api::HardwareParams & params,
  const std::vector<std::string> & joint_names,
  std::string & error)
{
  if (joint_names.empty()) {
    error = "IghHardwareBus requires at least one joint";
    return false;
  }

  params_ = params;
  joint_names_ = joint_names;

  master_ = std::make_unique<ethercat_master_igh::Master>(
    0, ethercat_master_igh::MotionPolicy::SupervisedMotion);
  if (!master_->init(error)) {
    master_.reset();
    return false;
  }

  std::vector<ethercat_master_igh::AxisConfig> axes;
  if (!build_axis_configs(joint_names_, axes, error)) {
    master_.reset();
    return false;
  }
  if (!master_->map_joints(axes, error)) {
    master_.reset();
    return false;
  }

  cmd_buf_.assign(joint_names_.size(), ethercat_master_igh::AxisCommand{});
  state_buf_.assign(joint_names_.size(), ethercat_master_igh::AxisState{});
  configured_ = true;
  error.clear();
  return true;
}

bool IghHardwareBus::start(std::string & error)
{
  if (!configured_ || !master_) {
    error = "IghHardwareBus not configured";
    return false;
  }
  if (!master_->start(error)) {
    return false;
  }
  std::string reset_err;
  if (!master_->request_safety_reset(reset_err)) {
    error = reset_err.empty() ? "post-start safety reset failed" : reset_err;
    return false;
  }
  running_ = true;
  error.clear();
  return true;
}

void IghHardwareBus::stop()
{
  running_ = false;
  if (master_) {
    master_->shutdown();
  }
}

bool IghHardwareBus::exchange(
  const robot_hardware_api::CommandSnapshot & command,
  robot_hardware_api::StateSnapshot & state,
  double /*dt_sec*/,  // PDO rate owned by master Job; not used for setpoint stride here.
  std::string & error)
{
  if (!running_ || !master_) {
    error = "IghHardwareBus is not running";
    return false;
  }

  const std::size_t n = joint_names_.size();
  if (command.position.size() != n ||
    command.velocity.size() != n ||
    command.effort.size() != n)
  {
    error = "Command size mismatch";
    return false;
  }

  for (std::size_t i = 0; i < n; ++i) {
    auto & c = cmd_buf_[i];
    c.position = command.position[i];
    c.velocity = command.velocity[i];
    c.effort = command.effort[i];
    // Empty / undersized enable → keep enabled (true). Undersized mode → 0 = unchanged.
    c.enable = (command.enable.size() == n) ? (command.enable[i] != 0) : true;
    c.operation_mode = (command.operation_mode.size() == n) ? command.operation_mode[i] : 0;
  }

  if (master_->motion_reenable_allowed()) {
    const auto health = master_->health();
    if (health.safe_output_active) {
      std::string release_err;
      (void)master_->release_safe_output(release_err);
    }
  }

  if (!master_->cycle(cmd_buf_.data(), n, state_buf_.data(), n, error)) {
    return false;
  }

  if (state.position.size() != n ||
    state.velocity.size() != n ||
    state.effort.size() != n)
  {
    error = "State size mismatch";
    return false;
  }
  // Lazy-size enabled/fault so callers that only resized pos/vel/effort still work.
  if (state.enabled.size() != n) {
    state.enabled.assign(n, 0);
  }
  if (state.fault.size() != n) {
    state.fault.assign(n, 0);
  }
  for (std::size_t i = 0; i < n; ++i) {
    state.position[i] = state_buf_[i].position;
    state.velocity[i] = state_buf_[i].velocity;
    state.effort[i] = state_buf_[i].effort;
    state.enabled[i] = state_buf_[i].enabled ? 1 : 0;
    state.fault[i] = state_buf_[i].fault ? 1 : 0;
  }

  error.clear();
  return true;
}

}  // namespace robot_ethercat_adapters

PLUGINLIB_EXPORT_CLASS(
  robot_ethercat_adapters::IghHardwareBus,
  robot_hardware_api::HardwareBus)
