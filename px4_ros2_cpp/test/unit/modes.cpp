/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/health_and_arming_checks.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include "fake_registration.hpp"

class TestMode : public px4_ros2::ModeBase
{
public:
  explicit TestMode(rclcpp::Node & node)
  : ModeBase(node, std::string("test"))
  {
    EXPECT_FALSE(modeRequirements().manual_control);
    EXPECT_FALSE(modeRequirements().angular_velocity);
    EXPECT_FALSE(modeRequirements().global_position);
    _manual_control_input = std::make_shared<px4_ros2::ManualControlInput>(*this);
    _rates_setpoint = std::make_shared<px4_ros2::RatesSetpointType>(*this);
    _global_position = std::make_shared<px4_ros2::OdometryGlobalPosition>(*this);
    EXPECT_TRUE(modeRequirements().manual_control);
    EXPECT_FALSE(modeRequirements().angular_velocity); // Setpoint requirements are only set after registration
    EXPECT_TRUE(modeRequirements().global_position);

    setSkipMessageCompatibilityCheck();
    overrideRegistration(std::make_shared<FakeRegistration>(node));
  }
  void onActivate() override {}
  void onDeactivate() override {}

private:
  std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input;
  std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;
  std::shared_ptr<px4_ros2::OdometryGlobalPosition> _global_position;
};

TEST(modes, modeRequirements)
{
  rclcpp::Node node("test_node");
  auto mode = std::make_shared<TestMode>(node);
  EXPECT_TRUE(mode->doRegister());
  EXPECT_TRUE(mode->modeRequirements().angular_velocity);

  mode->modeRequirements().clearAll();
  EXPECT_FALSE(mode->modeRequirements().angular_velocity);
}

TEST(modes, nodeWithMode)
{
  auto node_with_mode = std::make_shared<px4_ros2::NodeWithMode<TestMode>>("test_node");
  EXPECT_TRUE(node_with_mode->getMode().modeRequirements().angular_velocity);

  node_with_mode->getMode().modeRequirements().clearAll();
  EXPECT_FALSE(node_with_mode->getMode().modeRequirements().angular_velocity);
}
