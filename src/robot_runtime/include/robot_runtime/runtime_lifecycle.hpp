#pragma once

#include <memory>
#include <string>
#include <vector>

#include "robot_core_api/fault.hpp"
#include "robot_core_api/lifecycle.hpp"
#include "robot_core_api/status.hpp"

namespace robot_runtime
{

/// Tracks and drives a list of ILifecycle participants in order.
class RuntimeLifecycle
{
public:
  void add(std::shared_ptr<robot_core_api::ILifecycle> participant);

  robot_core_api::Status configure_all();
  robot_core_api::Status activate_all();
  robot_core_api::Status deactivate_all();
  robot_core_api::Status cleanup_all();

  robot_core_api::LifecycleState state() const { return state_; }

private:
  std::vector<std::shared_ptr<robot_core_api::ILifecycle>> participants_;
  robot_core_api::LifecycleState state_{robot_core_api::LifecycleState::Unconfigured};
};

}  // namespace robot_runtime
