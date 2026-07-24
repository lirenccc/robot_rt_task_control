#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "robot_interfaces/action/execute_manipulation.hpp"

using namespace std::chrono_literals;

class MockManipulationServer final : public rclcpp::Node
{
public:
  using ExecuteManipulation = robot_interfaces::action::ExecuteManipulation;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteManipulation>;

  MockManipulationServer()
  : Node("mock_manipulation_server")
  {
    server_ = rclcpp_action::create_server<ExecuteManipulation>(
      this,
      "/manipulation/execute_skill",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteManipulation::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread(&MockManipulationServer::execute, this, goal_handle).detach();
      });
  }

private:
  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const std::vector<std::string> phases{
      "detect object",
      "plan pregrasp",
      "execute approach",
      "close gripper",
      "lift object",
      "verify"
    };

    auto feedback = std::make_shared<ExecuteManipulation::Feedback>();

    for (std::size_t i = 0; i < phases.size(); ++i) {
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<ExecuteManipulation::Result>();
        result->success = false;
        result->message = "Manipulation canceled";
        goal_handle->canceled(result);
        return;
      }

      feedback->phase = phases[i];
      feedback->progress = static_cast<float>(i + 1) / static_cast<float>(phases.size());
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(200ms);
    }

    auto result = std::make_shared<ExecuteManipulation::Result>();
    result->success = true;
    result->message = "Mock manipulation complete";
    goal_handle->succeed(result);
  }

  rclcpp_action::Server<ExecuteManipulation>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockManipulationServer>());
  rclcpp::shutdown();
  return 0;
}
