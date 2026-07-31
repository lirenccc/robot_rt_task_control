/**
 * @brief 模块 robot_ethercat_adapters：框架侧薄 EtherCAT HardwareBus 适配。
 *
 * 将关节名 + ETHERCAT_MOTOR_MODEL 直接填入双主站公共扁平 AxisConfig。
 * 不含产品 Feature；不构造主站内部 MotorConfig（由 Master::map_joints 组装）。
 */

#pragma once

#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "ethercat_joint/motor/motor_profile.hpp"

namespace robot_ethercat_adapters
{
namespace detail
{

template<typename PdoLayoutT>
PdoLayoutT to_axis_pdo_layout(ethercat_joint::PdoLayout layout) noexcept
{
  switch (layout) {
    case ethercat_joint::PdoLayout::JOINT_MODULE:
      return PdoLayoutT::JointModule;
    case ethercat_joint::PdoLayout::GATEWAY:
      return PdoLayoutT::Gateway;
    case ethercat_joint::PdoLayout::UNKNOWN:
    default:
      return PdoLayoutT::Unknown;
  }
}

}  // namespace detail

/**
 * Build flat AxisConfig list for ethercat_master_ecmaster / ethercat_master_igh.
 * Slave position defaults to joint index (must match ENI / ACTIVE_DOF).
 * Default model NH17-100-BT-48E when ETHERCAT_MOTOR_MODEL is unset.
 */
template<typename AxisConfigT>
bool build_axis_configs(
  const std::vector<std::string> & joint_names,
  std::vector<AxisConfigT> & axes,
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
  using PdoLayoutT = decltype(AxisConfigT{}.pdo_layout);

  axes.clear();
  axes.reserve(joint_names.size());
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    AxisConfigT axis;
    axis.joint_name = joint_names[i];
    axis.alias = 0;
    axis.position = static_cast<uint16_t>(i);
    axis.vendor_id = id.vendor_id;
    axis.product_code = id.product_code;
    axis.model_id = profile->model_id;
    axis.pdo_layout = detail::to_axis_pdo_layout<PdoLayoutT>(profile->pdo_layout);
    axes.push_back(std::move(axis));
  }

  error.clear();
  return true;
}

}  // namespace robot_ethercat_adapters
