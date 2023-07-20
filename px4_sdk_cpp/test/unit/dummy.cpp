/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_sdk/components/message_compatibility_check.hpp>

TEST(dummy, aFirstTest)
{
  rclcpp::Node node("test_node");
  ASSERT_EQ(4, 2 + 2);
  ASSERT_FALSE(px4_sdk::messageCompatibilityCheck(node, {{"test"}}));
  //ASSERT_TRUE(false);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
