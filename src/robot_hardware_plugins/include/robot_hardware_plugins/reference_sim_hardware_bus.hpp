#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "robot_hardware_api/hardware_bus.hpp"

namespace robot_hardware_plugins
{

/// Production-shaped reference plant (no vendor bus): sequence, watchdog, fault inject.
class ReferenceSimHardwareBus final : public robot_hardware_api::HardwareBus
{
public:
  ReferenceSimHardwareBus() = default;
  ~ReferenceSimHardwareBus() override = default;

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
  bool inject_fault_{false};
  uint64_t sequence_{0};
  double time_since_command_sec_{0.0};
};

}  // namespace robot_hardware_plugins
