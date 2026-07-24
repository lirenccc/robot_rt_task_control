#include <gtest/gtest.h>

#include "robot_capability_api/ports.hpp"
#include "robot_core_api/status.hpp"

// Compile-time contract: joint targets are explicit; no IK fields required beyond pose.
TEST(ManipulationJointGoal, HasJointTargetHelpers)
{
  robot_capability_api::ManipulationGoal goal;
  EXPECT_FALSE(goal.has_joint_target());
  goal.joint_positions = {0.1, 0.2};
  EXPECT_TRUE(goal.has_joint_target());
  goal.joint_names = {"j1", "j2"};
  EXPECT_EQ(goal.joint_names.size(), goal.joint_positions.size());
}

TEST(ManipulationJointGoal, InvalidArgumentExists)
{
  EXPECT_STREQ(
    robot_core_api::to_string(robot_core_api::ErrorCode::InvalidArgument),
    "InvalidArgument");
}
