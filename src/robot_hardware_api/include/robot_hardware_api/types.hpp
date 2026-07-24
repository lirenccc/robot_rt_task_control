#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace robot_hardware_api
{

/// Marker for "no position setpoint this cycle". Buses should leave the axis
/// position target unchanged when they see NaN (finite checks skip the field).
inline double quiet_nan()
{
  return std::numeric_limits<double>::quiet_NaN();
}

/// Tunables shared by RtControlLoop and HardwareBus plugins.
/// position_kp / velocity_kd are consumed by plant-style buses (Mock/ReferenceSim),
/// not by the EtherCAT thin adapters (those forward setpoints only).
struct HardwareParams
{
  double loop_hz{1000.0};
  int rt_priority{70};
  /// When true, RtControlLoop requests SCHED_FIFO; failure is recorded but the loop still runs.
  bool use_fifo_scheduler{false};
  double max_velocity{2.0};
  double max_effort{20.0};
  double position_kp{8.0};
  double velocity_kd{0.15};
  double watchdog_timeout_sec{0.2};
  /// ReferenceSimHardwareBus: force exchange() failure after configure.
  bool sim_inject_fault{false};
};

/// Command pushed into HardwareBus::exchange each RT tick.
/// Vectors must match joint count for position/velocity/effort.
/// enable / operation_mode may be empty (= leave unchanged for all joints).
struct CommandSnapshot
{
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  /// Per-joint enable request (true=enable, false=disable). Empty = leave unchanged.
  std::vector<uint8_t> enable;
  /// CiA402 mode (e.g. 8=CSP, 10=CST). 0 = leave unchanged. Empty = leave unchanged.
  std::vector<int8_t> operation_mode;

  /// Allocates n joints. Defaults: enable=1 (request enable), operation_mode=0 (unchanged),
  /// position=NaN (no setpoint), velocity/effort=0.
  void resize(std::size_t n)
  {
    position.assign(n, quiet_nan());
    velocity.assign(n, 0.0);
    effort.assign(n, 0.0);
    enable.assign(n, 1);
    operation_mode.assign(n, 0);
  }
};

/// Feedback filled by HardwareBus::exchange. enabled/fault are per-joint bits from the bus.
struct StateSnapshot
{
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<uint8_t> enabled;
  std::vector<uint8_t> fault;

  void resize(std::size_t n)
  {
    position.assign(n, 0.0);
    velocity.assign(n, 0.0);
    effort.assign(n, 0.0);
    enabled.assign(n, 0);
    fault.assign(n, 0);
  }
};

/// Coarse bus/loop health for non-RT status queries (includes last_error string).
struct HardwareStatus
{
  bool running{false};
  double measured_frequency_hz{0.0};
  uint64_t loop_count{0};
  uint64_t missed_deadlines{0};
  std::string last_error;
};

/// Lock-free-friendly RT loop counters (no string) for ros2_control / topic publish.
struct RtStats
{
  bool running{false};
  double measured_frequency_hz{0.0};
  uint64_t loop_count{0};
  uint64_t missed_deadlines{0};
};

}  // namespace robot_hardware_api
