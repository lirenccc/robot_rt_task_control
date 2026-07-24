#include "robot_runtime/health_monitor.hpp"

using namespace std::chrono_literals;

namespace robot_runtime
{

HealthMonitor::HealthMonitor(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<robot_core_api::FaultManager> faults,
  std::shared_ptr<robot_capability_api::INavigationPort> navigation,
  std::shared_ptr<robot_capability_api::IManipulationPort> manipulation)
: node_(std::move(node)),
  faults_(std::move(faults)),
  navigation_(std::move(navigation)),
  manipulation_(std::move(manipulation))
{
}

robot_core_api::Status HealthMonitor::configure()
{
  if (!node_ || !faults_) {
    return robot_core_api::Status::error(
      robot_core_api::ErrorCode::InvalidConfiguration, "HealthMonitor missing dependencies");
  }
  pub_ = node_->create_publisher<robot_interfaces::msg::HardwareHealth>("/robot/health", 10);
  rt_sub_ = node_->create_subscription<robot_interfaces::msg::RtLoopStats>(
    "/robot/rt_loop_stats",
    rclcpp::SystemDefaultsQoS(),
    [this](const robot_interfaces::msg::RtLoopStats::SharedPtr msg) {
      if (msg) {
        on_rt_loop_stats(*msg);
      }
    });
  state_ = robot_core_api::LifecycleState::Inactive;
  return robot_core_api::Status::success();
}

robot_core_api::Status HealthMonitor::activate()
{
  timer_ = node_->create_wall_timer(200ms, [this]() { tick(); });
  state_ = robot_core_api::LifecycleState::Active;
  return robot_core_api::Status::success();
}

robot_core_api::Status HealthMonitor::deactivate()
{
  timer_.reset();
  state_ = robot_core_api::LifecycleState::Inactive;
  return robot_core_api::Status::success();
}

robot_core_api::Status HealthMonitor::cleanup()
{
  timer_.reset();
  rt_sub_.reset();
  pub_.reset();
  state_ = robot_core_api::LifecycleState::Finalized;
  return robot_core_api::Status::success();
}

void HealthMonitor::set_rt_stats(
  bool running,
  double measured_hz,
  uint64_t loop_count,
  uint64_t missed,
  std::string last_error)
{
  rt_running_ = running;
  measured_hz_ = measured_hz;
  loop_count_ = loop_count;
  missed_ = missed;
  last_error_ = std::move(last_error);
}

void HealthMonitor::on_rt_loop_stats(const robot_interfaces::msg::RtLoopStats & msg)
{
  set_rt_stats(
    msg.running,
    msg.measured_frequency_hz,
    msg.loop_count,
    msg.missed_deadlines,
    msg.last_error);

  if (!faults_) {
    return;
  }

  if (!msg.last_error.empty()) {
    faults_->raise(
      robot_core_api::FaultMode::Fault, "RT_LOOP_ERROR", msg.last_error);
    last_seen_missed_ = msg.missed_deadlines;
    have_missed_baseline_ = true;
    return;
  }

  if (have_missed_baseline_) {
    const uint64_t delta = msg.missed_deadlines >= last_seen_missed_
      ? msg.missed_deadlines - last_seen_missed_ : 0;
    if (delta >= kMissedDeltaFaultThreshold) {
      faults_->raise(
        robot_core_api::FaultMode::Fault,
        "RT_LOOP_MISSED",
        "missed_deadlines delta=" + std::to_string(delta));
    } else if (
      msg.running &&
      faults_->mode() == robot_core_api::FaultMode::Fault)
    {
      const auto code = faults_->fault_code();
      if (code == "RT_LOOP_ERROR" || code == "RT_LOOP_MISSED") {
        faults_->clear();
      }
    }
  }
  last_seen_missed_ = msg.missed_deadlines;
  have_missed_baseline_ = true;
}

void HealthMonitor::tick()
{
  if (!pub_ || !faults_) {
    return;
  }

  robot_interfaces::msg::HardwareHealth msg;
  msg.stamp = node_->now();
  msg.rt_loop_running = rt_running_;
  msg.measured_frequency_hz = measured_hz_;
  msg.loop_count = loop_count_;
  msg.missed_deadlines = missed_;
  msg.last_error = last_error_;
  if (msg.last_error.empty() && navigation_ && !navigation_->health().available) {
    msg.last_error = "navigation unavailable: " + navigation_->health().detail;
  }
  if (manipulation_ && !manipulation_->health().available) {
    if (!msg.last_error.empty()) {
      msg.last_error += "; ";
    }
    msg.last_error += "manipulation unavailable: " + manipulation_->health().detail;
  }
  msg.mode = robot_core_api::to_string(faults_->mode());
  msg.fault_code = faults_->fault_code();
  msg.estop_active = faults_->mode() == robot_core_api::FaultMode::Estop;
  msg.allows_motion = faults_->allows_motion();
  pub_->publish(msg);
}

}  // namespace robot_runtime
