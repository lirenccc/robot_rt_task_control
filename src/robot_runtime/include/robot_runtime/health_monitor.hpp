#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "robot_capability_api/ports.hpp"
#include "robot_core_api/fault.hpp"
#include "robot_core_api/lifecycle.hpp"
#include "robot_interfaces/msg/hardware_health.hpp"
#include "robot_interfaces/msg/rt_loop_stats.hpp"

namespace robot_runtime
{

/// Publishes /robot/health from FaultManager + capability availability + RT stats topic.
class HealthMonitor final : public robot_core_api::ILifecycle
{
public:
  HealthMonitor(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<robot_core_api::FaultManager> faults,
    std::shared_ptr<robot_capability_api::INavigationPort> navigation,
    std::shared_ptr<robot_capability_api::IManipulationPort> manipulation);

  robot_core_api::Status configure() override;
  robot_core_api::Status activate() override;
  robot_core_api::Status deactivate() override;
  robot_core_api::Status cleanup() override;
  robot_core_api::LifecycleState state() const override { return state_; }

  void set_rt_stats(
    bool running,
    double measured_hz,
    uint64_t loop_count,
    uint64_t missed,
    std::string last_error);

  /// Apply RT stats and update FaultManager (used by topic callback and tests).
  void on_rt_loop_stats(const robot_interfaces::msg::RtLoopStats & msg);

private:
  void tick();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<robot_core_api::FaultManager> faults_;
  std::shared_ptr<robot_capability_api::INavigationPort> navigation_;
  std::shared_ptr<robot_capability_api::IManipulationPort> manipulation_;
  rclcpp::Publisher<robot_interfaces::msg::HardwareHealth>::SharedPtr pub_;
  rclcpp::Subscription<robot_interfaces::msg::RtLoopStats>::SharedPtr rt_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  robot_core_api::LifecycleState state_{robot_core_api::LifecycleState::Unconfigured};

  bool rt_running_{false};
  double measured_hz_{0.0};
  uint64_t loop_count_{0};
  uint64_t missed_{0};
  uint64_t last_seen_missed_{0};
  bool have_missed_baseline_{false};
  std::string last_error_;

  static constexpr uint64_t kMissedDeltaFaultThreshold = 10;
};

}  // namespace robot_runtime
