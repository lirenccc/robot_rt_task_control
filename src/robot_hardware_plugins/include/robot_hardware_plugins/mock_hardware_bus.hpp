/**
 * @brief 模块 robot_hardware_plugins：内置 HardwareBus（Mock / Skeleton / ReferenceSim）。
 *
 * 无厂商主站即可跑通任务栈与 ros2_control；实机总线由外置主站包提供。
 */

#pragma once

#include <string>
#include <vector>

#include "robot_hardware_api/hardware_bus.hpp"

namespace robot_hardware_plugins
{

/// In-process plant for bring-up / CI without a vendor bus (integrates command into state).
class MockHardwareBus final : public robot_hardware_api::HardwareBus
{
public:
  MockHardwareBus() = default;
  ~MockHardwareBus() override = default;

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
  robot_hardware_api::StateSnapshot plant_state_;
  bool running_{false};
};

}  // namespace robot_hardware_plugins
