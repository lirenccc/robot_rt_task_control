#include "robot_runtime/runtime_lifecycle.hpp"

namespace robot_runtime
{

void RuntimeLifecycle::add(std::shared_ptr<robot_core_api::ILifecycle> participant)
{
  if (participant) {
    participants_.push_back(std::move(participant));
  }
}

robot_core_api::Status RuntimeLifecycle::configure_all()
{
  for (auto & p : participants_) {
    const auto st = p->configure();
    if (!st.ok()) {
      state_ = robot_core_api::LifecycleState::Error;
      return st;
    }
  }
  state_ = robot_core_api::LifecycleState::Inactive;
  return robot_core_api::Status::success();
}

robot_core_api::Status RuntimeLifecycle::activate_all()
{
  if (state_ != robot_core_api::LifecycleState::Inactive &&
    state_ != robot_core_api::LifecycleState::Unconfigured)
  {
    return robot_core_api::Status::error(
      robot_core_api::ErrorCode::InvalidConfiguration,
      "activate requires Inactive (or Unconfigured after configure)");
  }
  for (auto & p : participants_) {
    const auto st = p->activate();
    if (!st.ok()) {
      state_ = robot_core_api::LifecycleState::Error;
      return st;
    }
  }
  state_ = robot_core_api::LifecycleState::Active;
  return robot_core_api::Status::success();
}

robot_core_api::Status RuntimeLifecycle::deactivate_all()
{
  // Reverse order for safe shutdown
  for (auto it = participants_.rbegin(); it != participants_.rend(); ++it) {
    const auto st = (*it)->deactivate();
    if (!st.ok()) {
      state_ = robot_core_api::LifecycleState::Error;
      return st;
    }
  }
  state_ = robot_core_api::LifecycleState::Inactive;
  return robot_core_api::Status::success();
}

robot_core_api::Status RuntimeLifecycle::cleanup_all()
{
  for (auto it = participants_.rbegin(); it != participants_.rend(); ++it) {
    const auto st = (*it)->cleanup();
    if (!st.ok()) {
      state_ = robot_core_api::LifecycleState::Error;
      return st;
    }
  }
  state_ = robot_core_api::LifecycleState::Finalized;
  return robot_core_api::Status::success();
}

}  // namespace robot_runtime
