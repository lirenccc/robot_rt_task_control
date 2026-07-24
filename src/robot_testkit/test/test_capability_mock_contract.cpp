#include <gtest/gtest.h>

#include <atomic>

#include "robot_manipulation_adapters/manipulation_ports.hpp"
#include "robot_navigation_adapters/navigation_ports.hpp"

TEST(CapabilityMockContract, NavigationStartWaitCancel)
{
  robot_navigation_adapters::MockNavigationPort port;
  auto started = port.start({}, nullptr);
  ASSERT_TRUE(started.ok()) << started.status().message;
  EXPECT_TRUE(port.wait(started.value(), 1.0).ok());

  auto started2 = port.start({}, nullptr);
  ASSERT_TRUE(started2.ok());
  EXPECT_TRUE(port.cancel(started2.value()).ok());
}

TEST(CapabilityMockContract, ManipulationFeedbackAndWait)
{
  robot_manipulation_adapters::MockManipulationPort port;
  float last_progress = 0.0f;
  auto started = port.execute(
    {},
    [&](const robot_capability_api::ManipulationFeedback & fb) {
      last_progress = fb.progress;
    });
  ASSERT_TRUE(started.ok()) << started.status().message;
  EXPECT_TRUE(port.wait(started.value(), 1.0).ok());
  EXPECT_GT(last_progress, 0.0f);
  EXPECT_TRUE(port.health().available);
}
