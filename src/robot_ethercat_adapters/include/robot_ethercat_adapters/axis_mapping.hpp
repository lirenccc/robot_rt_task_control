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

/// Legacy AxisConfig: `{ joint_name, motor }` (IgH / pre-upgrade EC-Master).
template<typename AxisConfigT>
auto fill_axis_config(
  AxisConfigT & axis,
  const std::string & joint_name,
  const ethercat_joint::MotorConfig & motor,
  int) -> decltype(axis.motor = motor, void())
{
  axis.joint_name = joint_name;
  axis.motor = motor;
}

/// Flat AxisConfig: alias/position/VID/PID/model_id/pdo_layout (EC-Master upgrade).
template<typename AxisConfigT>
void fill_axis_config(
  AxisConfigT & axis,
  const std::string & joint_name,
  const ethercat_joint::MotorConfig & motor,
  long)
{
  axis.joint_name = joint_name;
  axis.alias = motor.alias;
  axis.position = motor.position;
  axis.vendor_id = motor.vendor_id;
  axis.product_code = motor.product_code;
  axis.model_id = motor.model_id;
  using PdoLayoutT = decltype(axis.pdo_layout);
  switch (motor.pdo_layout) {
    case ethercat_joint::PdoLayout::JOINT_MODULE:
      axis.pdo_layout = PdoLayoutT::JointModule;
      break;
    case ethercat_joint::PdoLayout::GATEWAY:
      axis.pdo_layout = PdoLayoutT::Gateway;
      break;
    case ethercat_joint::PdoLayout::UNKNOWN:
    default:
      axis.pdo_layout = PdoLayoutT::Unknown;
      break;
  }
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
    fill_axis_config(axis, joint_names[i], motors[i], 0);
    axes.push_back(axis);
  }
  return true;
}

}  // namespace robot_ethercat_adapters
