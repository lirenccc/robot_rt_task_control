#pragma once

#include <memory>
#include <string>

#include "robot_core_api/status.hpp"
#include "robot_hardware_api/types.hpp"

namespace robot_hardware_api
{

struct HardwareConfig
{
  std::string name;
  std::string plugin;
  HardwareParams params;
};

/// Lifecycle-facing hardware component. RT IO stays in HardwareBus::exchange.
class IHardwareComponent
{
public:
  virtual ~IHardwareComponent() = default;

  virtual robot_core_api::Status configure(const HardwareConfig & config) = 0;
  virtual robot_core_api::Status activate() = 0;
  virtual robot_core_api::Status deactivate() = 0;
  virtual robot_core_api::Status cleanup() = 0;
  virtual HardwareStatus status() const = 0;
};

}  // namespace robot_hardware_api
