#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "robot_interfaces/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class MockNavigationServer final : public rclcpp::Node
{
public:
  using NavigateToPose = robot_interfaces::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  MockNavigationServer()
  : Node("mock_navigation_server")
  {
    server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      "/navigation/navigate_to_pose",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread(&MockNavigationServer::execute, this, goal_handle).detach();
      });
  }

private:
  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    (void)goal_handle->get_goal();

    auto feedback = std::make_shared<NavigateToPose::Feedback>();

    for (int i = 0; i <= 10; ++i) {
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<NavigateToPose::Result>();
        result->success = false;
        result->message = "Navigation canceled";
        goal_handle->canceled(result);
        return;
      }

      feedback->phase = i < 3 ? "global planning" : "tracking path";
      feedback->progress = static_cast<float>(i) / 10.0f;
      feedback->distance_remaining = static_cast<float>(10 - i) * 0.1f;
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(150ms);
    }

    auto result = std::make_shared<NavigateToPose::Result>();
    result->success = true;
    result->message = "Mock navigation reached target";
    goal_handle->succeed(result);
  }

  rclcpp_action::Server<NavigateToPose>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockNavigationServer>());
  rclcpp::shutdown();
  return 0;
}
