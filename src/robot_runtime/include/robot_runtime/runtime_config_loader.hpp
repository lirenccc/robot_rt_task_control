#pragma once

#include <string>

#include "robot_core_api/result.hpp"
#include "robot_runtime/runtime_config.hpp"

namespace robot_runtime
{

class RuntimeConfigLoader
{
public:
  robot_core_api::Result<RuntimeConfig> load_file(const std::string & path) const;
  robot_core_api::Result<RuntimeConfig> load_string(const std::string & yaml_text) const;
};

}  // namespace robot_runtime
