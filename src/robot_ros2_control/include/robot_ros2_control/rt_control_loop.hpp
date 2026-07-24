/**
 * @brief 模块 robot_ros2_control：ros2_control SystemInterface + 独立 RT 控制环。
 *
 * 将 HardwareBus 插件挂到 controller_manager；RT 线程周期 exchange，
 * 非 RT 侧经 RealtimeBuffer / AtomicStateBuffer 桥接。
 */

#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <realtime_tools/realtime_buffer.h>

#include "robot_ros2_control/atomic_state_buffer.hpp"
#include "robot_ros2_control/hardware_bus.hpp"
#include "robot_ros2_control/safety_limiter.hpp"
#include "robot_ros2_control/types.hpp"

namespace robot_ros2_control
{

/// Owns a dedicated RT thread that calls HardwareBus::exchange at loop_hz.
///
/// Threading:
/// - write_command_from_non_rt / read_state_from_non_rt / stats: controller_manager or other non-RT.
/// - run(): RT thread only; uses RealtimeBuffer for commands and AtomicStateBuffer for state.
/// - last_error uses a mutex (failure path only; not on the happy path).
///
/// Frequency is independent of controller_manager update rate: write() only publishes the latest
/// command snapshot; the RT thread may consume the same command for several ticks.
class RtControlLoop
{
public:
  RtControlLoop(
    HardwareParams params,
    std::vector<std::string> joint_names,
    std::shared_ptr<HardwareBus> bus);

  ~RtControlLoop();

  RtControlLoop(const RtControlLoop &) = delete;
  RtControlLoop & operator=(const RtControlLoop &) = delete;

  bool configure(std::string & error);
  bool start(std::string & error);
  void stop();

  void write_command_from_non_rt(const CommandSnapshot & command);
  StateSnapshot read_state_from_non_rt() const;
  RtStats stats() const;
  std::string last_error() const;

private:
  void run();
  void try_set_realtime_priority();
  static std::chrono::steady_clock::time_point next_tick(
    const std::chrono::steady_clock::time_point & start,
    uint64_t tick,
    std::chrono::nanoseconds period);

  HardwareParams params_;
  std::vector<std::string> joint_names_;
  std::shared_ptr<HardwareBus> bus_;
  SafetyLimiter limiter_;

  realtime_tools::RealtimeBuffer<CommandSnapshot> command_buffer_;
  AtomicStateBuffer state_buffer_;

  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<uint64_t> loop_count_{0};
  std::atomic<uint64_t> missed_deadlines_{0};
  std::atomic<double> measured_frequency_hz_{0.0};

  mutable std::mutex error_mutex_;
  std::string last_error_;
};

}  // namespace robot_ros2_control
