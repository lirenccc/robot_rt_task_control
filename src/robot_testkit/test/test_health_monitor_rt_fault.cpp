#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "robot_core_api/fault.hpp"
#include "robot_interfaces/msg/rt_loop_stats.hpp"
#include "robot_runtime/health_monitor.hpp"

class HealthMonitorRtFaultTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // Avoid writing under ~/.ros when the environment is read-only.
    setenv("ROS_LOG_DIR", "/tmp/robot_testkit_ros_log", 1);
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_health_monitor_rt");
    faults_ = std::make_shared<robot_core_api::FaultManager>();
    monitor_ = std::make_shared<robot_runtime::HealthMonitor>(
      node_, faults_, nullptr, nullptr);
    ASSERT_TRUE(monitor_->configure().ok());
    ASSERT_TRUE(monitor_->activate().ok());
  }

  void TearDown() override
  {
    (void)monitor_->deactivate();
    (void)monitor_->cleanup();
    monitor_.reset();
    faults_.reset();
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<robot_core_api::FaultManager> faults_;
  std::shared_ptr<robot_runtime::HealthMonitor> monitor_;
};

TEST_F(HealthMonitorRtFaultTest, RtErrorRaisesFaultAndBlocksMotion)
{
  robot_interfaces::msg::RtLoopStats msg;
  msg.running = true;
  msg.measured_frequency_hz = 1000.0;
  msg.loop_count = 10;
  msg.missed_deadlines = 0;
  msg.last_error = "exchange failed";

  monitor_->on_rt_loop_stats(msg);

  EXPECT_EQ(faults_->mode(), robot_core_api::FaultMode::Fault);
  EXPECT_EQ(faults_->fault_code(), "RT_LOOP_ERROR");
  EXPECT_FALSE(faults_->allows_motion());
}

TEST_F(HealthMonitorRtFaultTest, MissedDeltaRaisesFault)
{
  robot_interfaces::msg::RtLoopStats base;
  base.running = true;
  base.missed_deadlines = 5;
  monitor_->on_rt_loop_stats(base);
  EXPECT_TRUE(faults_->allows_motion());

  robot_interfaces::msg::RtLoopStats spike;
  spike.running = true;
  spike.missed_deadlines = 5 + 10;
  monitor_->on_rt_loop_stats(spike);

  EXPECT_EQ(faults_->mode(), robot_core_api::FaultMode::Fault);
  EXPECT_EQ(faults_->fault_code(), "RT_LOOP_MISSED");
  EXPECT_FALSE(faults_->allows_motion());
}

TEST_F(HealthMonitorRtFaultTest, HealthyStatsClearRtFault)
{
  robot_interfaces::msg::RtLoopStats bad;
  bad.running = true;
  bad.last_error = "bus timeout";
  monitor_->on_rt_loop_stats(bad);
  ASSERT_FALSE(faults_->allows_motion());

  robot_interfaces::msg::RtLoopStats ok;
  ok.running = true;
  ok.missed_deadlines = bad.missed_deadlines;
  ok.last_error = "";
  monitor_->on_rt_loop_stats(ok);

  EXPECT_EQ(faults_->mode(), robot_core_api::FaultMode::Ok);
  EXPECT_TRUE(faults_->allows_motion());
}
