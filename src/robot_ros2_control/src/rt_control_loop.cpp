#include "robot_ros2_control/rt_control_loop.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

#ifdef __linux__
  #include <pthread.h>
  #include <sched.h>
#endif

namespace robot_ros2_control
{

RtControlLoop::RtControlLoop(
  HardwareParams params,
  std::vector<std::string> joint_names,
  std::shared_ptr<HardwareBus> bus)
: params_(params),
  joint_names_(std::move(joint_names)),
  bus_(std::move(bus)),
  limiter_(params_)
{
  CommandSnapshot initial_command;
  initial_command.resize(joint_names_.size());
  StateSnapshot initial_state;
  initial_state.resize(joint_names_.size());

  command_buffer_.writeFromNonRT(initial_command);
  state_buffer_.resize(joint_names_.size());
  state_buffer_.write_from_rt(initial_state);
}

RtControlLoop::~RtControlLoop()
{
  stop();
}

bool RtControlLoop::configure(std::string & error)
{
  if (!bus_) {
    error = "No HardwareBus instance";
    return false;
  }

  if (params_.loop_hz <= 0.0) {
    error = "loop_hz must be positive";
    return false;
  }

  return bus_->configure(params_, joint_names_, error);
}

bool RtControlLoop::start(std::string & error)
{
  if (running_.load()) {
    error.clear();
    return true;
  }

  if (!bus_->start(error)) {
    return false;
  }

  stop_requested_.store(false);
  running_.store(true);
  loop_count_.store(0);
  missed_deadlines_.store(0);
  measured_frequency_hz_.store(0.0);

  thread_ = std::thread(&RtControlLoop::run, this);
  error.clear();
  return true;
}

void RtControlLoop::stop()
{
  stop_requested_.store(true);

  if (thread_.joinable()) {
    thread_.join();
  }

  if (bus_) {
    bus_->stop();
  }

  running_.store(false);
}

void RtControlLoop::write_command_from_non_rt(const CommandSnapshot & command)
{
  command_buffer_.writeFromNonRT(command);
}

StateSnapshot RtControlLoop::read_state_from_non_rt() const
{
  return state_buffer_.read_from_non_rt();
}

RtStats RtControlLoop::stats() const
{
  RtStats s;
  s.running = running_.load();
  s.measured_frequency_hz = measured_frequency_hz_.load();
  s.loop_count = loop_count_.load();
  s.missed_deadlines = missed_deadlines_.load();
  return s;
}

std::string RtControlLoop::last_error() const
{
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

void RtControlLoop::try_set_realtime_priority()
{
#ifdef __linux__
  // Soft request only: this loop keeps running under CFS if SCHED_FIFO is denied
  // (common without CAP_SYS_NICE / rtprio). Priority is clamped to 1–98 to leave 99
  // for critical system threads. (EtherCAT masters may refuse start when RT setup fails.)
  if (!params_.use_fifo_scheduler) {
    return;
  }

  sched_param sched{};
  sched.sched_priority = std::clamp(params_.rt_priority, 1, 98);

  const int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sched);
  if (ret != 0) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = std::string("Failed to set SCHED_FIFO: ") + std::strerror(ret);
  }
#endif
}

std::chrono::steady_clock::time_point RtControlLoop::next_tick(
  const std::chrono::steady_clock::time_point & start,
  uint64_t tick,
  std::chrono::nanoseconds period)
{
  return start + period * static_cast<int64_t>(tick);
}

void RtControlLoop::run()
{
  // Absolute-time schedule from a fixed start: tick starts at 1 so the first wake is one
  // period after start (avoids an immediate oversleep race). Missed-deadline counts only the
  // work after wake (limiter + exchange + state publish), not sleep_until latency. A failed
  // exchange records last_error but does not stop the loop — fault policy lives in the bus /
  // master Job (safe-output), not here. Null command_ptr keeps the previous command.
  try_set_realtime_priority();

  const auto period_ns = static_cast<int64_t>(1e9 / params_.loop_hz);
  const auto period = std::chrono::nanoseconds(period_ns);
  const double dt_sec = 1.0 / params_.loop_hz;

  CommandSnapshot command;
  command.resize(joint_names_.size());

  StateSnapshot state;
  state.resize(joint_names_.size());

  auto start = std::chrono::steady_clock::now();
  auto last_measure = start;
  uint64_t last_count = 0;

  uint64_t tick = 1;

  while (!stop_requested_.load(std::memory_order_relaxed)) {
    const auto wake_time = next_tick(start, tick, period);
    std::this_thread::sleep_until(wake_time);

    const auto loop_begin = std::chrono::steady_clock::now();

    auto * command_ptr = command_buffer_.readFromRT();
    if (command_ptr) {
      command = *command_ptr;
    }

    limiter_.limit(command);

    std::string error;
    const bool ok = bus_->exchange(command, state, dt_sec, error);
    if (!ok) {
      std::lock_guard<std::mutex> lock(error_mutex_);
      last_error_ = error;
    }

    state_buffer_.write_from_rt(state);

    const auto loop_end = std::chrono::steady_clock::now();
    const auto elapsed = loop_end - loop_begin;
    if (elapsed > period) {
      missed_deadlines_.fetch_add(1, std::memory_order_relaxed);
    }

    const auto total_count = loop_count_.fetch_add(1, std::memory_order_relaxed) + 1;

    const auto measure_elapsed = loop_end - last_measure;
    if (measure_elapsed >= std::chrono::seconds(1)) {
      const double sec = std::chrono::duration<double>(measure_elapsed).count();
      const double hz = static_cast<double>(total_count - last_count) / sec;
      measured_frequency_hz_.store(hz, std::memory_order_relaxed);
      last_measure = loop_end;
      last_count = total_count;
    }

    ++tick;
  }
}

}  // namespace robot_ros2_control
