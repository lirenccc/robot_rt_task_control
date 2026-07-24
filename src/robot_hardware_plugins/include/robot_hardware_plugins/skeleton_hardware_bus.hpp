#pragma once

#include <string>
#include <vector>

#include "robot_hardware_api/hardware_bus.hpp"

namespace robot_hardware_plugins
{

/// Vendor-agnostic HardwareBus skeleton for real robots.
/// Fork this class: replace TODO sections with bus IO. Do not expose frames/registers upward.
/// exchange() must stay allocation-free and non-blocking.
class SkeletonHardwareBus final : public robot_hardware_api::HardwareBus
{
public:
  SkeletonHardwareBus() = default;
  ~SkeletonHardwareBus() override = default;

  bool configure(
    const robot_hardware_api::HardwareParams & params,
    const std::vector<std::string> & joint_names,
    std::string & error) override;

  bool start(std::string & error) override;
  void stop() override;

  bool exchange(
    const robot_hardware_api::CommandSnapshot & command,
    robot_hardware_api::StateSnapshot & state,
    double dt_sec,
    std::string & error) override;

private:
  robot_hardware_api::HardwareParams params_;
  std::vector<std::string> joint_names_;
  robot_hardware_api::StateSnapshot last_state_;
  bool running_{false};
  bool transport_ready_{false};
};

}  // namespace robot_hardware_plugins
