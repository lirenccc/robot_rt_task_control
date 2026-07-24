#include <atomic>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "robot_runtime/runtime_builder.hpp"
#include "robot_task/planner.hpp"
#include "robot_interfaces/action/task.hpp"
#include "robot_interfaces/msg/task_state.hpp"
#include "robot_interfaces/srv/set_task_mode.hpp"

using namespace std::chrono_literals;

class TaskOrchestratorNode final : public rclcpp::Node
{
public:
  using Task = robot_interfaces::action::Task;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Task>;

  TaskOrchestratorNode()
  : Node("task_orchestrator_node")
  {
    declare_parameter<std::string>("profile_file", "");
    declare_parameter<std::string>("runtime_file", "");
  }

  void initialize()
  {
    const auto profile_file = get_parameter("profile_file").as_string();
    const auto runtime_file = get_parameter("runtime_file").as_string();
    if (profile_file.empty() || runtime_file.empty()) {
      throw std::runtime_error("profile_file and runtime_file parameters are required");
    }

    robot_runtime::RuntimeBuilder builder;
    auto built = builder.build_from_files(profile_file, runtime_file, shared_from_this());
    if (!built.ok()) {
      throw std::runtime_error(built.status().message);
    }
    runtime_ = std::make_shared<robot_runtime::RobotRuntime>(std::move(built.value()));

    const auto configured = runtime_->configure();
    if (!configured.ok()) {
      throw std::runtime_error("Runtime configure failed: " + configured.message);
    }
    const auto activated = runtime_->activate();
    if (!activated.ok()) {
      throw std::runtime_error("Runtime activate failed: " + activated.message);
    }

    state_pub_ = create_publisher<robot_interfaces::msg::TaskState>("/task/state", 10);
    mode_srv_ = create_service<robot_interfaces::srv::SetTaskMode>(
      "/task/set_mode",
      [this](
        const std::shared_ptr<robot_interfaces::srv::SetTaskMode::Request> request,
        std::shared_ptr<robot_interfaces::srv::SetTaskMode::Response> response)
      {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        mode_ = request->mode;
        response->accepted = true;
        response->message = "Mode set to " + mode_;
      });

    task_server_ = rclcpp_action::create_server<Task>(
      this,
      "/task/execute",
      std::bind(&TaskOrchestratorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TaskOrchestratorNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&TaskOrchestratorNode::handle_accepted, this, std::placeholders::_1));

    state_timer_ = create_wall_timer(200ms, [this]() { publish_state(); });
    set_state("IDLE", "", "", 0.0f);
    RCLCPP_INFO(get_logger(), "Task orchestrator ready (profile=%s)", profile_file.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Task::Goal> goal)
  {
    if (goal->instruction.empty()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ != "IDLE") {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    cancel_requested_.store(true);
    set_state("CANCELING", active_goal_, "cancel requested", progress_);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    cancel_requested_.store(false);
    const auto instruction = goal_handle->get_goal()->instruction;
    set_state("PLANNING", instruction, "parse instruction", 0.05f);

    robot_task::InstructionParser parser;
    auto request = parser.parse(instruction);
    if (!request.ok()) {
      finish_failed(goal_handle, request.status().message);
      return;
    }

    auto status = runtime_->orchestrator->execute(
      request.value(),
      [this, goal_handle, instruction](const robot_task::OrchestratorProgress & p) {
        set_state(p.state, instruction, p.step, p.progress);
        if (!goal_handle->is_canceling()) {
          auto fb = std::make_shared<Task::Feedback>();
          fb->state = p.state;
          fb->current_step = p.step;
          fb->progress = p.progress;
          goal_handle->publish_feedback(fb);
        }
      },
      cancel_requested_);

    if (status.code == robot_core_api::ErrorCode::Canceled || goal_handle->is_canceling()) {
      auto result = std::make_shared<Task::Result>();
      result->success = false;
      result->message = "Task canceled";
      goal_handle->canceled(result);
      set_state("IDLE", "", "", 0.0f);
      return;
    }

    if (!status.ok()) {
      finish_failed(goal_handle, status.message);
      return;
    }

    auto result = std::make_shared<Task::Result>();
    result->success = true;
    result->message = status.message.empty() ? "Task completed" : status.message;
    goal_handle->succeed(result);
    set_state("IDLE", "", "", 0.0f);
  }

  void finish_failed(const std::shared_ptr<GoalHandle> goal_handle, const std::string & message)
  {
    set_state("ERROR", active_goal_, message, progress_);
    auto result = std::make_shared<Task::Result>();
    result->success = false;
    result->message = message;
    goal_handle->abort(result);
    set_state("IDLE", "", "", 0.0f);
  }

  void publish_state()
  {
    robot_interfaces::msg::TaskState msg;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      msg.stamp = now();
      msg.state = state_;
      msg.active_goal = active_goal_;
      msg.active_step = active_step_;
      msg.progress = progress_;
    }
    state_pub_->publish(msg);
  }

  void set_state(
    const std::string & state,
    const std::string & active_goal,
    const std::string & active_step,
    float progress)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = state;
    active_goal_ = active_goal;
    active_step_ = active_step;
    progress_ = progress;
  }

  std::shared_ptr<robot_runtime::RobotRuntime> runtime_;
  std::atomic<bool> cancel_requested_{false};
  std::mutex state_mutex_;
  std::string state_;
  std::string active_goal_;
  std::string active_step_;
  float progress_{0.0f};
  std::mutex mode_mutex_;
  std::string mode_{"auto"};

  rclcpp::Publisher<robot_interfaces::msg::TaskState>::SharedPtr state_pub_;
  rclcpp::Service<robot_interfaces::srv::SetTaskMode>::SharedPtr mode_srv_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp_action::Server<Task>::SharedPtr task_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskOrchestratorNode>();
  try {
    node->initialize();
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(node->get_logger(), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
