#pragma once

#include <string>

namespace robot_core_api
{

enum class ErrorCode
{
  Ok = 0,
  InvalidConfiguration,
  InvalidArgument,
  Unavailable,
  RejectedBySafety,
  Timeout,
  Canceled,
  CommunicationError,
  HardwareFault,
  InternalError
};

inline const char * to_string(ErrorCode code)
{
  switch (code) {
    case ErrorCode::Ok: return "Ok";
    case ErrorCode::InvalidConfiguration: return "InvalidConfiguration";
    case ErrorCode::InvalidArgument: return "InvalidArgument";
    case ErrorCode::Unavailable: return "Unavailable";
    case ErrorCode::RejectedBySafety: return "RejectedBySafety";
    case ErrorCode::Timeout: return "Timeout";
    case ErrorCode::Canceled: return "Canceled";
    case ErrorCode::CommunicationError: return "CommunicationError";
    case ErrorCode::HardwareFault: return "HardwareFault";
    case ErrorCode::InternalError: return "InternalError";
  }
  return "Unknown";
}

struct Status
{
  ErrorCode code{ErrorCode::Ok};
  std::string message;
  bool retryable{false};

  static Status success(std::string message = {})
  {
    return Status{ErrorCode::Ok, std::move(message), false};
  }

  static Status error(ErrorCode code, std::string message, bool retryable = false)
  {
    return Status{code, std::move(message), retryable};
  }

  bool ok() const { return code == ErrorCode::Ok; }
};

}  // namespace robot_core_api
