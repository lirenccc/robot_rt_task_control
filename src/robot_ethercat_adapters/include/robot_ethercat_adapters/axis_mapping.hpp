/**
 * @brief 模块 robot_ethercat_adapters：框架侧薄 EtherCAT HardwareBus 适配。
 *
 * 将关节名/电机型号映射到外置主站 Master API；不含产品 Feature（拖动/轨迹）逻辑。
 */

#pragma once

#include <cstdlib>
#include <string>
#include <vector>

#include "ethercat_joint/motor/motor_profile.hpp"
#include "ethercat_joint/servo/ethercat_servo.hpp"

namespace robot_ethercat_adapters
{

/// Build MotorConfig list from joint_names + ETHERCAT_MOTOR_MODEL (no hard-coded SKU in callers).
/// Slave position defaults to joint index order (must match ENI / ACTIVE_DOF on the bench;
/// URDF may still list 7 links while only the first N slaves exist).
/// Default model NH17-100-BT-48E when the env var is unset; VID/PID/PDO come from MotorProfileRegistry.
inline bool build_motor_configs(
  const std::vector<std::string> & joint_names,
  std::vector<ethercat_joint::MotorConfig> & motors,
  std::string & error)
{
  if (joint_names.empty()) {
    error = "joint_names empty";
    return false;
  }

  const char * model_env = std::getenv("ETHERCAT_MOTOR_MODEL");
  const std::string model_id =
    (model_env != nullptr && model_env[0] != '\0') ? model_env : "NH17-100-BT-48E";

  const auto * profile = ethercat_joint::MotorProfileRegistry::findByModelId(model_id);
  if (profile == nullptr || profile->identities.empty()) {
    error = "Unknown ETHERCAT_MOTOR_MODEL: " + model_id;
    return false;
  }

  const auto & id = profile->identities.front();
  motors.clear();
  motors.reserve(joint_names.size());
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    ethercat_joint::MotorConfig cfg;
    cfg.alias = 0;
    cfg.position = static_cast<uint16_t>(i);
    cfg.vendor_id = id.vendor_id;
    cfg.product_code = id.product_code;
    cfg.name = joint_names[i];
    cfg.model_id = profile->model_id;
    cfg.pdo_layout = profile->pdo_layout;
    motors.push_back(cfg);
  }

  error.clear();
  return true;
}

template<typename AxisConfigT>
bool build_axis_configs(
  const std::vector<std::string> & joint_names,
  std::vector<AxisConfigT> & axes,
  std::string & error)
{
  std::vector<ethercat_joint::MotorConfig> motors;
  if (!build_motor_configs(joint_names, motors, error)) {
    return false;
  }
  axes.clear();
  axes.reserve(motors.size());
  for (std::size_t i = 0; i < motors.size(); ++i) {
    AxisConfigT axis;
    axis.joint_name = joint_names[i];
    axis.motor = motors[i];
    axes.push_back(axis);
  }
  return true;
}

}  // namespace robot_ethercat_adapters
