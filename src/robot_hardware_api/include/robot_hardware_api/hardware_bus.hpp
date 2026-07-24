/**
 * @brief 模块 robot_hardware_api：硬件总线抽象（pluginlib 基类）。
 *
 * 提供 HardwareBus 与命令/状态快照，隔离产品与运输层；实时路径约定见 exchange()。
 */

#pragma once

#include <string>
#include <vector>

#include "robot_hardware_api/types.hpp"

namespace robot_hardware_api
{

/// RT-oriented bus exchange boundary. Implementations must avoid allocation/blocking in exchange().
/// Loaded via pluginlib; do not expose transport frames (CAN/EtherCAT) at this layer.
///
/// Call order: configure() → start() → exchange()* → stop().
/// configure/start/stop are non-RT; exchange() runs on the RT loop thread.
/// On failure, write a human-readable message into `error` (may allocate); callers must not
/// treat a failed exchange as an automatic stop of the RT loop.
class HardwareBus
{
public:
  virtual ~HardwareBus() = default;

  /// Bind joint names and allocate steady-path buffers. Must not start cyclic IO.
  virtual bool configure(
    const HardwareParams & params,
    const std::vector<std::string> & joint_names,
    std::string & error) = 0;

  /// Bring the transport to operational / start cyclic ownership as needed by the backend.
  virtual bool start(std::string & error) = 0;
  /// Stop cyclic IO and release transport resources. Safe to call multiple times.
  virtual void stop() = 0;

  /// One RT tick: apply `command`, fill `state`. `dt_sec` is the nominal loop period;
  /// thin EtherCAT adapters may ignore it (PDO rate owned by the master Job).
  virtual bool exchange(
    const CommandSnapshot & command,
    StateSnapshot & state,
    double dt_sec,
    std::string & error) = 0;
};

}  // namespace robot_hardware_api
