#include <gtest/gtest.h>

#ifdef ROBOT_HAS_NAV2
#include "robot_navigation_adapters/nav2_navigation_port.hpp"
#endif

TEST(Nav2Adapter, CompiledWhenNav2MsgsPresent)
{
#ifdef ROBOT_HAS_NAV2
  SUCCEED() << "Nav2NavigationPort is compiled in";
#else
  GTEST_SKIP() << "nav2_msgs not found at build time; install and rebuild to enable";
#endif
}
