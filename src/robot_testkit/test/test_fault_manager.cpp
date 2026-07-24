#include <gtest/gtest.h>

#include "robot_core_api/fault.hpp"

TEST(FaultManager, DefaultAllowsMotion)
{
  robot_core_api::FaultManager fm;
  EXPECT_TRUE(fm.allows_motion());
  EXPECT_TRUE(fm.assert_motion_allowed().ok());
}

TEST(FaultManager, FaultBlocksMotion)
{
  robot_core_api::FaultManager fm;
  fm.raise(robot_core_api::FaultMode::Fault, "TEST", "boom");
  EXPECT_FALSE(fm.allows_motion());
  EXPECT_FALSE(fm.assert_motion_allowed().ok());
  fm.clear();
  EXPECT_TRUE(fm.allows_motion());
}

TEST(FaultManager, EstopBlocksMotion)
{
  robot_core_api::FaultManager fm;
  fm.raise(robot_core_api::FaultMode::Estop, "ESTOP", "pressed");
  EXPECT_EQ(fm.mode(), robot_core_api::FaultMode::Estop);
  EXPECT_FALSE(fm.allows_motion());
}
