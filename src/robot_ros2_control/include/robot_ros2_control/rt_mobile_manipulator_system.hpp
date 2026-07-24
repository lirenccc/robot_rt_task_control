#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "robot_hardware_api/hardware_bus.hpp"
#include "robot_interfaces/msg/rt_loop_stats.hpp"
#include "robot_ros2_control/rt_control_loop.hpp"
#include "robot_ros2_control/types.hpp"

namespace robot_ros2_control
{

/// ros2_control SystemInterface that hosts RtControlLoop + a pluginlib HardwareBus.
///
/// read()/write() run on the controller_manager update thread (non-RT relative to the
/// dedicated RT loop): they only bridge AtomicStateBuffer / RealtimeBuffer. Command
/// interfaces are position/velocity/effort only — enable and operation_mode are not exported
/// here; product FeatureBridge / Arm*HardwareBus overlay those on the same process RT path.
class RtMobileManipulatorSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RtMobileManipulatorSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  HardwareParams parse_params() const;
  std::string hardware_bus_plugin() const;
  void maybe_publish_rt_stats(const rclcpp::Time & time);
  static bool parse_bool(const std::string & value, bool fallback);
  static double parse_double(const std::string & value, double fallback);
  static int parse_int(const std::string & value, int fallback);

  std::vector<std::string> joint_names_;
  CommandSnapshot command_;
  StateSnapshot state_;
  std::unique_ptr<pluginlib::ClassLoader<robot_hardware_api::HardwareBus>> bus_loader_;
  std::shared_ptr<robot_hardware_api::HardwareBus> bus_;
  std::unique_ptr<RtControlLoop> rt_loop_;
  bool configured_{false};

  rclcpp::Node::SharedPtr stats_node_;
  rclcpp::Publisher<robot_interfaces::msg::RtLoopStats>::SharedPtr stats_pub_;
  rclcpp::Time last_stats_pub_{0, 0, RCL_ROS_TIME};
};

}  // namespace robot_ros2_control
