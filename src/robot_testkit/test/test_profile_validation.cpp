#include <gtest/gtest.h>

#include "robot_profile/loader.hpp"

TEST(ProfileValidation, AcceptsValidExample)
{
  const char * yaml = R"(
schema_version: 1
robot:
  model: mobile_manipulator
  variant: standard
  serial_number: robot_001
capabilities:
  navigation: true
  manipulation: true
  gripper: true
frames:
  base: base_link
  map: map
  tool: tool0
limits:
  max_linear_velocity: 1.0
  max_angular_velocity: 1.5
  payload_kg: 5.0
)";
  robot_profile::ProfileLoader loader;
  const auto result = loader.load_string(yaml);
  ASSERT_TRUE(result.ok()) << result.status().message;
  EXPECT_EQ(result.value().robot.model, "mobile_manipulator");
}

TEST(ProfileValidation, RejectsMissingSchema)
{
  const char * yaml = R"(
robot:
  model: x
frames:
  base: base_link
  map: map
limits:
  max_linear_velocity: 1.0
  max_angular_velocity: 1.0
  payload_kg: 1.0
)";
  robot_profile::ProfileLoader loader;
  const auto result = loader.load_string(yaml);
  EXPECT_FALSE(result.ok());
}

TEST(ProfileValidation, AcceptsJointsAndMechanism)
{
  const char * yaml = R"(
schema_version: 1
robot:
  model: parallel_mechanism
  variant: 6ucu
  serial_number: pm_001
capabilities:
  navigation: false
  manipulation: true
  gripper: false
frames:
  base: base_link
  map: map
  tool: platform_link
limits:
  max_linear_velocity: 0.2
  max_angular_velocity: 0.5
  payload_kg: 20.0
joints:
  names: [a, b, c]
  unit: meter
  type: prismatic
mechanism:
  plugin: pm_6ucu
)";
  robot_profile::ProfileLoader loader;
  const auto result = loader.load_string(yaml);
  ASSERT_TRUE(result.ok()) << result.status().message;
  EXPECT_EQ(result.value().joints.names.size(), 3u);
  EXPECT_EQ(result.value().mechanism_plugin(), "pm_6ucu");
}

TEST(ProfileValidation, RejectsBadJointUnit)
{
  const char * yaml = R"(
schema_version: 1
robot:
  model: x
frames:
  base: base_link
  map: map
limits:
  max_linear_velocity: 1.0
  max_angular_velocity: 1.0
  payload_kg: 1.0
joints:
  names: [j1]
  unit: mm
  type: prismatic
)";
  robot_profile::ProfileLoader loader;
  const auto result = loader.load_string(yaml);
  EXPECT_FALSE(result.ok());
}
