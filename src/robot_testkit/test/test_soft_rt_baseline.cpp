#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include <pluginlib/class_loader.hpp>

#include "robot_hardware_api/hardware_bus.hpp"
#include "robot_ros2_control/rt_control_loop.hpp"

TEST(SoftRtBaseline, MockBusHoldsThousandHz)
{
  pluginlib::ClassLoader<robot_hardware_api::HardwareBus> loader(
    "robot_hardware_api", "robot_hardware_api::HardwareBus");
  auto bus = loader.createSharedInstance("robot_hardware_plugins/MockHardwareBus");
  ASSERT_NE(bus, nullptr);

  robot_hardware_api::HardwareParams params;
  params.loop_hz = 1000.0;
  params.use_fifo_scheduler = false;
  params.watchdog_timeout_sec = 1.0;

  robot_ros2_control::RtControlLoop loop(params, {"j1", "j2"}, bus);
  std::string error;
  ASSERT_TRUE(loop.configure(error)) << error;
  ASSERT_TRUE(loop.start(error)) << error;

  std::this_thread::sleep_for(std::chrono::milliseconds(2200));

  const auto stats = loop.stats();
  loop.stop();

  ASSERT_TRUE(stats.running || stats.loop_count > 0);
  EXPECT_GT(stats.loop_count, 1500u) << "expected ~2000 cycles at 1 kHz";
  // Soft RT: allow tiny misses on loaded CI hosts (< 0.1% of ~2000).
  EXPECT_LE(stats.missed_deadlines, 2u) << "missed=" << stats.missed_deadlines;
  EXPECT_GE(stats.measured_frequency_hz, 900.0);
  EXPECT_LE(stats.measured_frequency_hz, 1100.0);
}
