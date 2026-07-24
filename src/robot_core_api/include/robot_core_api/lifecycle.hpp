#pragma once

#include "robot_core_api/status.hpp"

namespace robot_core_api
{

enum class LifecycleState
{
  Unconfigured,
  Inactive,
  Active,
  Finalized,
  Error
};

/// Shared lifecycle contract for hardware and capability components.
class ILifecycle
{
public:
  virtual ~ILifecycle() = default;

  virtual Status configure() = 0;
  virtual Status activate() = 0;
  virtual Status deactivate() = 0;
  virtual Status cleanup() = 0;
  virtual LifecycleState state() const = 0;
};

}  // namespace robot_core_api
