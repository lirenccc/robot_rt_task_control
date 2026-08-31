/**
 * @brief 模块 robot_ethercat_adapters：框架侧薄 EtherCAT HardwareBus 适配。
 *
 * 将关节名 + ETHERCAT_MOTOR_MODEL（及 MOTORS_CONFIG_FILE 中 motor_model /
 * motor_models[]）填入双主站公共扁平 AxisConfig。
 * 不含产品 Feature；不构造主站内部 MotorConfig（由 Master::map_joints 组装）。
 */

#pragma once

#include <cstdlib>
#include <fstream>
#include <regex>
#include <sstream>
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
  // 薄 CSP 适配只转发关节/驱动布局；非驱动从站（如六维力）由产品 Arm*HardwareBus 追加。
  switch (layout) {
    case ethercat_joint::PdoLayout::JOINT_MODULE:
      return PdoLayoutT::JointModule;
    case ethercat_joint::PdoLayout::GATEWAY:
      return PdoLayoutT::Gateway;
    case ethercat_joint::PdoLayout::COOLDRIVE_JMDT:
      return PdoLayoutT::CoolDriveJmdt;
    case ethercat_joint::PdoLayout::UNKNOWN:
    default:
      return PdoLayoutT::Unknown;
  }
}

inline std::string read_motors_config_text()
{
  const char * path_env = std::getenv("MOTORS_CONFIG_FILE");
  if (path_env == nullptr || path_env[0] == '\0') {
    return {};
  }
  std::ifstream in(path_env);
  if (!in) {
    return {};
  }
  std::ostringstream oss;
  oss << in.rdbuf();
  return oss.str();
}

inline std::string parse_yaml_quoted_string(const std::string & text, const char * key)
{
  const std::regex re(std::string(key) + "\\s*:\\s*\"([^\"]*)\"");
  std::smatch m;
  if (std::regex_search(text, m, re)) {
    return m[1].str();
  }
  return {};
}

inline std::vector<std::string> parse_yaml_string_list(const std::string & text, const char * key)
{
  std::vector<std::string> out;
  const std::regex block_re(std::string(key) + "\\s*:\\s*\\[([^\\]]*)\\]");
  std::smatch block;
  if (!std::regex_search(text, block, block_re)) {
    return out;
  }
  const std::regex item_re("\"([^\"]*)\"");
  const auto begin = std::sregex_iterator(block[1].first, block[1].second, item_re);
  const auto end = std::sregex_iterator();
  for (auto it = begin; it != end; ++it) {
    out.push_back((*it)[1].str());
  }
  return out;
}

}  // namespace detail

/**
 * Build flat AxisConfig list for ethercat_master_ecmaster / ethercat_master_igh.
 * Slave position defaults to joint index (must match ENI / ACTIVE_DOF).
 * Model id required: ETHERCAT_MOTOR_MODEL and/or MOTORS_CONFIG_FILE
 * (`motor_model`, optional per-axis `motor_models[]`). No vendor model hardcoded here.
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
  std::string default_model =
    (model_env != nullptr && model_env[0] != '\0') ? model_env : std::string{};

  const std::string yaml_text = detail::read_motors_config_text();
  if (!yaml_text.empty()) {
    const std::string yaml_model = detail::parse_yaml_quoted_string(yaml_text, "motor_model");
    if (!yaml_model.empty()) {
      default_model = yaml_model;
    }
  }

  const auto per_axis_models = detail::parse_yaml_string_list(yaml_text, "motor_models");
  using PdoLayoutT = decltype(AxisConfigT{}.pdo_layout);

  axes.clear();
  axes.reserve(joint_names.size());
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    std::string model_id = default_model;
    if (i < per_axis_models.size() && !per_axis_models[i].empty()) {
      model_id = per_axis_models[i];
    }
    if (model_id.empty()) {
      error = "motor model required for axis " + std::to_string(i) +
        " (set ETHERCAT_MOTOR_MODEL or MOTORS_CONFIG_FILE motor_model / motor_models[])";
      return false;
    }

    const auto * profile = ethercat_joint::MotorProfileRegistry::findByModelId(model_id);
    if (profile == nullptr || profile->identities.empty()) {
      error = "Unknown motor model for axis " + std::to_string(i) + ": " + model_id;
      return false;
    }

    const auto & id = profile->identities.front();
    // isKnownMotor 排除非驱动从站；力传感器等不得经薄 CSP 适配按关节映射。
    if (!ethercat_joint::MotorProfileRegistry::isKnownMotor(id.vendor_id, id.product_code)) {
      error = "Non-drive EtherCAT slave model not supported by thin CSP adapter"
        " (use product Arm*HardwareBus): " + model_id;
      return false;
    }

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
