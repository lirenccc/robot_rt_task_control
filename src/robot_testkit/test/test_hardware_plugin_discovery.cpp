#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>
#include <vector>

#include "robot_hardware_api/hardware_bus.hpp"

TEST(HardwarePluginDiscovery, LoadsMockHardwareBus)
{
  pluginlib::ClassLoader<robot_hardware_api::HardwareBus> loader(
    "robot_hardware_api", "robot_hardware_api::HardwareBus");

  auto bus = loader.createSharedInstance("robot_hardware_plugins/MockHardwareBus");
  ASSERT_NE(bus, nullptr);

  robot_hardware_api::HardwareParams params;
  std::vector<std::string> joints{"j1", "j2"};
  std::string error;
  ASSERT_TRUE(bus->configure(params, joints, error)) << error;
  ASSERT_TRUE(bus->start(error)) << error;

  robot_hardware_api::CommandSnapshot cmd;
  cmd.resize(2);
  robot_hardware_api::StateSnapshot state;
  state.resize(2);
  ASSERT_TRUE(bus->exchange(cmd, state, 0.001, error)) << error;
  bus->stop();
}

TEST(HardwarePluginDiscovery, LoadsSkeletonHardwareBus)
{
  pluginlib::ClassLoader<robot_hardware_api::HardwareBus> loader(
    "robot_hardware_api", "robot_hardware_api::HardwareBus");

  auto bus = loader.createSharedInstance("robot_hardware_plugins/SkeletonHardwareBus");
  ASSERT_NE(bus, nullptr);

  robot_hardware_api::HardwareParams params;
  std::vector<std::string> joints{"j1"};
  std::string error;
  ASSERT_TRUE(bus->configure(params, joints, error)) << error;
  ASSERT_TRUE(bus->start(error)) << error;

  robot_hardware_api::CommandSnapshot cmd;
  cmd.resize(1);
  cmd.position[0] = 1.25;
  robot_hardware_api::StateSnapshot state;
  state.resize(1);
  ASSERT_TRUE(bus->exchange(cmd, state, 0.001, error)) << error;
  EXPECT_DOUBLE_EQ(state.position[0], 1.25);
  bus->stop();
}

TEST(HardwarePluginDiscovery, LoadsReferenceSimHardwareBus)
{
  pluginlib::ClassLoader<robot_hardware_api::HardwareBus> loader(
    "robot_hardware_api", "robot_hardware_api::HardwareBus");

  auto bus = loader.createSharedInstance("robot_hardware_plugins/ReferenceSimHardwareBus");
  ASSERT_NE(bus, nullptr);

  robot_hardware_api::HardwareParams params;
  params.watchdog_timeout_sec = 1.0;
  std::vector<std::string> joints{"j1"};
  std::string error;
  ASSERT_TRUE(bus->configure(params, joints, error)) << error;
  ASSERT_TRUE(bus->start(error)) << error;

  robot_hardware_api::CommandSnapshot cmd;
  cmd.resize(1);
  cmd.position[0] = 0.5;
  robot_hardware_api::StateSnapshot state;
  state.resize(1);
  ASSERT_TRUE(bus->exchange(cmd, state, 0.001, error)) << error;
  EXPECT_DOUBLE_EQ(state.position[0], 0.5);
  EXPECT_GT(state.effort[0], 0.0);

  params.sim_inject_fault = true;
  ASSERT_TRUE(bus->configure(params, joints, error)) << error;
  ASSERT_TRUE(bus->start(error)) << error;
  EXPECT_FALSE(bus->exchange(cmd, state, 0.001, error));
  bus->stop();
}
