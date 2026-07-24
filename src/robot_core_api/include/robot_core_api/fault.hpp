#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include "robot_core_api/status.hpp"

namespace robot_core_api
{

enum class FaultMode
{
  Ok = 0,
  Degraded,
  Fault,
  Estop
};

inline const char * to_string(FaultMode mode)
{
  switch (mode) {
    case FaultMode::Ok: return "OK";
    case FaultMode::Degraded: return "DEGRADED";
    case FaultMode::Fault: return "FAULT";
    case FaultMode::Estop: return "ESTOP";
  }
  return "UNKNOWN";
}

/// Process-wide fault / mode gate (software). Hardware e-stop remains below this layer.
class FaultManager
{
public:
  FaultMode mode() const { return mode_.load(); }

  std::string fault_code() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return fault_code_;
  }

  std::string detail() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return detail_;
  }

  bool allows_motion() const
  {
    const auto m = mode_.load();
    return m == FaultMode::Ok || m == FaultMode::Degraded;
  }

  Status raise(FaultMode mode, std::string code, std::string detail)
  {
    if (mode == FaultMode::Ok) {
      return Status::error(ErrorCode::InvalidConfiguration, "Use clear() to return to Ok");
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      fault_code_ = std::move(code);
      detail_ = std::move(detail);
    }
    mode_.store(mode);
    return Status::success();
  }

  Status clear()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      fault_code_.clear();
      detail_.clear();
    }
    mode_.store(FaultMode::Ok);
    return Status::success();
  }

  Status assert_motion_allowed() const
  {
    if (!allows_motion()) {
      return Status::error(
        ErrorCode::RejectedBySafety,
        std::string("Motion blocked: mode=") + to_string(mode()) +
          " code=" + fault_code());
    }
    return Status::success();
  }

private:
  std::atomic<FaultMode> mode_{FaultMode::Ok};
  mutable std::mutex mutex_;
  std::string fault_code_;
  std::string detail_;
};

}  // namespace robot_core_api
