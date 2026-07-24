#pragma once

#include <optional>
#include <utility>

#include "robot_core_api/status.hpp"

namespace robot_core_api
{

template<typename T>
class Result
{
public:
  static Result success(T value)
  {
    Result r;
    r.value_ = std::move(value);
    r.status_ = Status::success();
    return r;
  }

  static Result failure(Status status)
  {
    Result r;
    r.status_ = std::move(status);
    return r;
  }

  static Result failure(ErrorCode code, std::string message, bool retryable = false)
  {
    return failure(Status::error(code, std::move(message), retryable));
  }

  bool ok() const { return status_.ok() && value_.has_value(); }
  const Status & status() const { return status_; }
  const T & value() const { return *value_; }
  T & value() { return *value_; }

private:
  std::optional<T> value_;
  Status status_;
};

}  // namespace robot_core_api
