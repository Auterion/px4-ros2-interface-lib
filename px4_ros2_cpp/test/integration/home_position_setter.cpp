/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>

#include <px4_ros2/common/context.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <px4_ros2/vehicle_state/home_position_setter.hpp>
#include <rclcpp/rclcpp.hpp>

#include "util.hpp"

using namespace std::chrono_literals;

TEST_F(ModesTest, HomePositionSetter)
{
  auto test_node = initNode();
  ASSERT_TRUE(px4_ros2::waitForFMU(*test_node, 10s, 5s));

  px4_ros2::Context context(*test_node);
  px4_ros2::HomePositionSetter setter(context);

  // Set GPS global origin (EKF reference point)
  auto result = setter.setGpsGlobalOrigin(47.397742, 8.545594, 488.f);
  EXPECT_EQ(result, px4_ros2::Result::Success)
      << "setGpsGlobalOrigin failed: " << px4_ros2::resultToString(result);

  // Set home to current vehicle position
  result = setter.setHomeToCurrentPosition();
  EXPECT_EQ(result, px4_ros2::Result::Success)
      << "setHomeToCurrentPosition failed: " << px4_ros2::resultToString(result);

  // Set home to specific global coordinates
  result = setter.setHome(47.398, 8.546, 490.f);
  EXPECT_EQ(result, px4_ros2::Result::Success)
      << "setHome failed: " << px4_ros2::resultToString(result);
}
