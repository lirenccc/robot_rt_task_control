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

TEST(ProfileValidation, RejectsNonPositiveVelocityLimit)
{
  const char * yaml = R"(
schema_version: 1
robot:
  model: x
frames:
  base: base_link
  map: map
limits:
  max_linear_velocity: 0.0
  max_angular_velocity: 1.0
  payload_kg: 1.0
)";
  robot_profile::ProfileLoader loader;
  const auto result = loader.load_string(yaml);
  EXPECT_FALSE(result.ok());
}
