#pragma once

#include <memory>
#include <string>
#include <vector>

#include "ethercat_master_ecmaster/master_api.hpp"
#include "robot_hardware_api/hardware_bus.hpp"

namespace robot_ethercat_adapters
{

/// Thin HardwareBus over ethercat_master_ecmaster::Master.
/// exchange() uses pre-sized buffers (no steady-path heap allocation).
/// dt_sec is ignored: cyclic PDO ownership lives in the EC-Master Job thread.
/// When command.enable / operation_mode size != joint count, defaults are enable=true and
/// operation_mode=0 (leave mode unchanged) so ros2_control-only command paths keep axes armed.
class EcMasterHardwareBus final : public robot_hardware_api::HardwareBus
{
public:
  EcMasterHardwareBus() = default;
  ~EcMasterHardwareBus() override;

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
  std::unique_ptr<ethercat_master_ecmaster::Master> master_;
  std::vector<ethercat_master_ecmaster::AxisCommand> cmd_buf_;
  std::vector<ethercat_master_ecmaster::AxisState> state_buf_;
  bool configured_{false};
  bool running_{false};
};

}  // namespace robot_ethercat_adapters
