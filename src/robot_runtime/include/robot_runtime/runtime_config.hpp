#pragma once

#include <string>
#include <vector>

namespace robot_runtime
{

struct HardwareRuntimeEntry
{
  std::string name;
  std::string plugin;
};

struct ProviderRuntimeEntry
{
  std::string type;       // ros_action | mock | nav2 | ...
  std::string endpoint;
};

struct PlannerRuntimeEntry
{
  std::string type{"simple"};  // simple | yaml_graph
  std::string tasks_dir;       // directory of *.yaml graphs
};

struct RuntimeConfig
{
  int schema_version{0};
  HardwareRuntimeEntry hardware_base;
  ProviderRuntimeEntry navigation;
  ProviderRuntimeEntry manipulation;
  PlannerRuntimeEntry planner;
  std::vector<std::string> safety_policies;
};

}  // namespace robot_runtime
