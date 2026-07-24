#include "robot_task/task_orchestrator.hpp"

namespace robot_task
{

TaskOrchestrator::TaskOrchestrator(
  std::shared_ptr<robot_capability_api::INavigationPort> navigation,
  std::shared_ptr<robot_capability_api::IManipulationPort> manipulation,
  std::shared_ptr<robot_safety::SafetyGate> safety_gate,
  std::shared_ptr<robot_core_api::FaultManager> faults,
  std::shared_ptr<ITaskPlanner> planner)
: navigation_(std::move(navigation)),
  manipulation_(std::move(manipulation)),
  safety_gate_(std::move(safety_gate)),
  faults_(std::move(faults)),
  planner_(std::move(planner))
{
}

robot_core_api::Status TaskOrchestrator::execute(
  const TaskRequest & request,
  ProgressCallback progress,
  const std::atomic<bool> & cancel_requested)
{
  auto emit = [&](const std::string & state, const std::string & step, float p) {
    if (progress) {
      progress(OrchestratorProgress{state, step, p});
    }
  };

  if (!planner_) {
    return robot_core_api::Status::error(
      robot_core_api::ErrorCode::InvalidConfiguration, "Task planner is null");
  }

  if (faults_) {
    const auto gate = faults_->assert_motion_allowed();
    if (!gate.ok()) {
      return gate;
    }
  }

  emit("PLANNING", "plan task graph", 0.05f);
  const auto graph_result = planner_->plan(request);
  if (!graph_result.ok()) {
    return graph_result.status();
  }
  const auto & graph = graph_result.value();

  robot_safety::RobotState robot_state;
  robot_state.state_known = true;
  robot_state.estop_active = faults_ && faults_->mode() == robot_core_api::FaultMode::Estop;

  const std::size_t n = graph.steps.size();
  for (std::size_t i = 0; i < n; ++i) {
    if (cancel_requested.load()) {
      return robot_core_api::Status::error(robot_core_api::ErrorCode::Canceled, "Task canceled");
    }
    if (faults_) {
      const auto gate = faults_->assert_motion_allowed();
      if (!gate.ok()) {
        return gate;
      }
    }

    const auto & step = graph.steps[i];
    const float base = static_cast<float>(i) / static_cast<float>(n);
    const float span = 1.0f / static_cast<float>(n);

    robot_safety::RobotCommand cmd;
    cmd.label = step.label;
    if (step.kind == TaskStepKind::Navigate) {
      cmd.kind = robot_safety::CommandKind::Navigate;
      cmd.requested_linear_velocity = 0.5;
      cmd.requested_angular_velocity = 0.3;
    } else {
      cmd.kind = robot_safety::CommandKind::Manipulate;
    }

    const auto admit = safety_gate_->admit(cmd, robot_state);
    if (!admit.allowed()) {
      if (faults_) {
        faults_->raise(robot_core_api::FaultMode::Fault, "SAFETY_DENY", admit.reason);
      }
      return robot_core_api::Status::error(
        robot_core_api::ErrorCode::RejectedBySafety, admit.reason);
    }

    if (step.kind == TaskStepKind::Navigate) {
      emit("NAVIGATING", step.label, base + 0.1f * span);
      robot_capability_api::NavigateGoal goal;
      goal.target_pose = step.pose;
      auto start = navigation_->start(
        goal,
        [&](const robot_capability_api::NavigationFeedback & fb) {
          emit("NAVIGATING", fb.phase, base + span * (0.1f + 0.8f * fb.progress));
          robot_safety::ActiveCommand active{cmd, ""};
          (void)safety_gate_->monitor(active, robot_state);
        });
      if (!start.ok()) {
        if (faults_) {
          faults_->raise(robot_core_api::FaultMode::Fault, "NAV_FAIL", start.status().message);
        }
        return start.status();
      }
      if (cancel_requested.load()) {
        navigation_->cancel(start.value());
        return robot_core_api::Status::error(robot_core_api::ErrorCode::Canceled, "Task canceled");
      }
      const auto waited = navigation_->wait(start.value(), 30.0);
      if (!waited.ok()) {
        if (faults_) {
          faults_->raise(robot_core_api::FaultMode::Fault, "NAV_FAIL", waited.message);
        }
        return waited;
      }
    } else {
      emit("MANIPULATING", step.label, base + 0.1f * span);
      robot_capability_api::ManipulationGoal goal;
      goal.skill_name = step.skill_name;
      goal.object_id = step.object_id;
      goal.target_pose = step.pose;
      goal.joint_names = step.joint_names;
      goal.joint_positions = step.joint_positions;
      goal.joint_velocities = step.joint_velocities;
      goal.duration_sec = step.duration_sec;
      auto start = manipulation_->execute(
        goal,
        [&](const robot_capability_api::ManipulationFeedback & fb) {
          emit("MANIPULATING", fb.phase, base + span * (0.1f + 0.8f * fb.progress));
        });
      if (!start.ok()) {
        if (faults_) {
          faults_->raise(robot_core_api::FaultMode::Fault, "MANIP_FAIL", start.status().message);
        }
        return start.status();
      }
      if (cancel_requested.load()) {
        manipulation_->cancel(start.value());
        return robot_core_api::Status::error(robot_core_api::ErrorCode::Canceled, "Task canceled");
      }
      const auto waited = manipulation_->wait(start.value(), 30.0);
      if (!waited.ok()) {
        if (faults_) {
          faults_->raise(robot_core_api::FaultMode::Fault, "MANIP_FAIL", waited.message);
        }
        return waited;
      }
    }
  }

  emit("DONE", "task complete", 1.0f);
  return robot_core_api::Status::success("Task completed");
}

}  // namespace robot_task
