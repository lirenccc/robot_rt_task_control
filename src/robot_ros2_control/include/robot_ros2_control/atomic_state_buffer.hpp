#pragma once

#include <atomic>
#include <cstddef>
#include <vector>

#include "robot_ros2_control/types.hpp"

namespace robot_ros2_control
{

// Direction: RT thread writes latest hardware state, non-RT ros2_control read() copies it.
// This avoids a mutex in the RT loop. On some platforms atomic<double> may not be lock-free;
// production builds should verify this with std::atomic<double>::is_always_lock_free or
// replace this with a platform-specific lock-free shared memory/ring-buffer primitive.
//
// Only position/velocity/effort are mirrored here: enabled/fault stay on StateSnapshot for the
// RT writer path and are not yet needed by framework state interfaces (pos/vel/effort only).
class AtomicStateBuffer
{
public:
  AtomicStateBuffer() = default;

  void resize(std::size_t n)
  {
    position_ = std::vector<std::atomic<double>>(n);
    velocity_ = std::vector<std::atomic<double>>(n);
    effort_ = std::vector<std::atomic<double>>(n);

    for (std::size_t i = 0; i < n; ++i) {
      position_[i].store(0.0, std::memory_order_relaxed);
      velocity_[i].store(0.0, std::memory_order_relaxed);
      effort_[i].store(0.0, std::memory_order_relaxed);
    }
  }

  void write_from_rt(const StateSnapshot & state)
  {
    const std::size_t n = position_.size();
    for (std::size_t i = 0; i < n; ++i) {
      position_[i].store(state.position[i], std::memory_order_release);
      velocity_[i].store(state.velocity[i], std::memory_order_release);
      effort_[i].store(state.effort[i], std::memory_order_release);
    }
  }

  StateSnapshot read_from_non_rt() const
  {
    StateSnapshot out;
    out.resize(position_.size());

    for (std::size_t i = 0; i < position_.size(); ++i) {
      out.position[i] = position_[i].load(std::memory_order_acquire);
      out.velocity[i] = velocity_[i].load(std::memory_order_acquire);
      out.effort[i] = effort_[i].load(std::memory_order_acquire);
    }

    return out;
  }

private:
  std::vector<std::atomic<double>> position_;
  std::vector<std::atomic<double>> velocity_;
  std::vector<std::atomic<double>> effort_;
};

}  // namespace robot_ros2_control
