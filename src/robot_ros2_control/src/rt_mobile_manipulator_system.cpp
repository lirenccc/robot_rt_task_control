#include "robot_ros2_control/rt_mobile_manipulator_system.hpp"

#include <cstdlib>
#include <memory>
#include <string>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robot_ros2_control
{

hardware_interface::CallbackReturn RtMobileManipulatorSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  joint_names_.reserve(info_.joints.size());

  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }

  if (joint_names_.empty()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  command_.resize(joint_names_.size());
  state_.resize(joint_names_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RtMobileManipulatorSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joint_names_.size() * 3);

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &state_.position[i]);
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &state_.velocity[i]);
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_EFFORT, &state_.effort[i]);
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
RtMobileManipulatorSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joint_names_.size() * 3);

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &command_.position[i]);
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &command_.velocity[i]);
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_EFFORT, &command_.effort[i]);
  }

  return interfaces;
}

hardware_interface::CallbackReturn RtMobileManipulatorSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  const HardwareParams params = parse_params();
  const std::string plugin_name = hardware_bus_plugin();

  try {
    bus_loader_ = std::make_unique<pluginlib::ClassLoader<robot_hardware_api::HardwareBus>>(
      "robot_hardware_api", "robot_hardware_api::HardwareBus");
    bus_ = bus_loader_->createSharedInstance(plugin_name);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      rclcpp::get_logger("RtMobileManipulatorSystem"),
      "Failed to load HardwareBus plugin '%s': %s", plugin_name.c_str(), ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  rt_loop_ = std::make_unique<RtControlLoop>(params, joint_names_, bus_);

  std::string error;
  if (!rt_loop_->configure(error)) {
    RCLCPP_FATAL(
      rclcpp::get_logger("RtMobileManipulatorSystem"),
      "RT loop configure failed: %s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  stats_node_ = std::make_shared<rclcpp::Node>("rt_loop_stats_publisher");
  stats_pub_ = stats_node_->create_publisher<robot_interfaces::msg::RtLoopStats>(
    "/robot/rt_loop_stats", rclcpp::SystemDefaultsQoS());
  // Do not seed with Node::now() — controller_manager may use Steady clock.
  last_stats_pub_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);

  configured_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RtMobileManipulatorSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!configured_ || !rt_loop_) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string error;
  if (!rt_loop_->start(error)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RtMobileManipulatorSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (rt_loop_) {
    rt_loop_->stop();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RtMobileManipulatorSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (rt_loop_) {
    rt_loop_->stop();
    rt_loop_.reset();
  }
  bus_.reset();
  bus_loader_.reset();
  stats_pub_.reset();
  stats_node_.reset();

  configured_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RtMobileManipulatorSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  // Non-RT: copy the latest RT-published state into hardware_interface buffers.
  if (!rt_loop_) {
    return hardware_interface::return_type::ERROR;
  }

  const auto latest = rt_loop_->read_state_from_non_rt();

  if (latest.position.size() != state_.position.size()) {
    return hardware_interface::return_type::ERROR;
  }

  state_ = latest;
  maybe_publish_rt_stats(time);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RtMobileManipulatorSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Non-RT: push command interfaces into the RT RealtimeBuffer (may be sampled many times).
  // enable/operation_mode are not part of these interfaces; see class comment.
  if (!rt_loop_) {
    return hardware_interface::return_type::ERROR;
  }

  rt_loop_->write_command_from_non_rt(command_);
  return hardware_interface::return_type::OK;
}

HardwareParams RtMobileManipulatorSystem::parse_params() const
{
  HardwareParams p;

  const auto & hp = info_.hardware_parameters;

  auto get = [&](const std::string & key) -> std::string {
    const auto it = hp.find(key);
    return it == hp.end() ? std::string{} : it->second;
  };

  p.loop_hz = parse_double(get("rt_loop_hz"), p.loop_hz);
  p.rt_priority = parse_int(get("rt_priority"), p.rt_priority);
  p.use_fifo_scheduler = parse_bool(get("use_fifo_scheduler"), p.use_fifo_scheduler);
  p.max_velocity = parse_double(get("max_velocity"), p.max_velocity);
  p.max_effort = parse_double(get("max_effort"), p.max_effort);
  p.position_kp = parse_double(get("position_kp"), p.position_kp);
  p.velocity_kd = parse_double(get("velocity_kd"), p.velocity_kd);
  p.watchdog_timeout_sec = parse_double(get("watchdog_timeout_sec"), p.watchdog_timeout_sec);
  p.sim_inject_fault = parse_bool(get("sim_inject_fault"), p.sim_inject_fault);

  return p;
}

std::string RtMobileManipulatorSystem::hardware_bus_plugin() const
{
  const auto it = info_.hardware_parameters.find("hardware_bus_plugin");
  if (it == info_.hardware_parameters.end() || it->second.empty()) {
    return "robot_hardware_plugins/MockHardwareBus";
  }
  return it->second;
}

void RtMobileManipulatorSystem::maybe_publish_rt_stats(const rclcpp::Time & time)
{
  if (!stats_pub_ || !rt_loop_) {
    return;
  }
  if (last_stats_pub_.get_clock_type() != time.get_clock_type()) {
    last_stats_pub_ = rclcpp::Time(0, 0, time.get_clock_type());
  }
  // ~10 Hz from the non-RT controller_manager update path.
  if ((time - last_stats_pub_).seconds() < 0.1) {
    return;
  }
  last_stats_pub_ = time;

  const auto s = rt_loop_->stats();
  robot_interfaces::msg::RtLoopStats msg;
  msg.stamp = time;
  msg.running = s.running;
  msg.measured_frequency_hz = s.measured_frequency_hz;
  msg.loop_count = s.loop_count;
  msg.missed_deadlines = s.missed_deadlines;
  msg.last_error = rt_loop_->last_error();
  stats_pub_->publish(msg);
}

bool RtMobileManipulatorSystem::parse_bool(const std::string & value, bool fallback)
{
  if (value == "true" || value == "1" || value == "yes") {
    return true;
  }
  if (value == "false" || value == "0" || value == "no") {
    return false;
  }
  return fallback;
}

double RtMobileManipulatorSystem::parse_double(const std::string & value, double fallback)
{
  if (value.empty()) {
    return fallback;
  }

  char * end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str()) {
    return fallback;
  }
  return parsed;
}

int RtMobileManipulatorSystem::parse_int(const std::string & value, int fallback)
{
  if (value.empty()) {
    return fallback;
  }

  char * end = nullptr;
  const long parsed = std::strtol(value.c_str(), &end, 10);
  if (end == value.c_str()) {
    return fallback;
  }
  return static_cast<int>(parsed);
}

}  // namespace robot_ros2_control

PLUGINLIB_EXPORT_CLASS(
  robot_ros2_control::RtMobileManipulatorSystem,
  hardware_interface::SystemInterface)
