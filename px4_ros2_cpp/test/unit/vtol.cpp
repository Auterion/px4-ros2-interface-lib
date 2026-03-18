/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>

#include <px4_ros2/control/vtol.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace std::chrono_literals;

class VTOLTest : public testing::Test {
 protected:
  void SetUp() override
  {
    _node = std::make_shared<rclcpp::Node>("vtol_test_node");
    _executor.add_node(_node);
  }

  bool publishVtolStatus(px4_ros2::VTOL& vtol, uint8_t state)
  {
    auto pub = _node->create_publisher<px4_msgs::msg::VtolVehicleStatus>(
        "fmu/out/vtol_vehicle_status", rclcpp::QoS(10).best_effort());
    px4_msgs::msg::VtolVehicleStatus msg;
    msg.vehicle_vtol_state = state;
    pub->publish(msg);

    // Poll until subscription delivers the message (or timeout)
    const auto expected = toExpectedState(state);
    const auto start = _node->get_clock()->now();
    while (_node->get_clock()->now() - start < 3s) {
      _executor.spin_some();
      if (vtol.getCurrentState() == expected) {
        return true;
      }
      std::this_thread::yield();
    }
    return false;
  }

  static px4_ros2::VTOL::State toExpectedState(uint8_t state)
  {
    switch (state) {
      case px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_MC:
        return px4_ros2::VTOL::State::Multicopter;
      case px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_FW:
        return px4_ros2::VTOL::State::FixedWing;
      case px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_TRANSITION_TO_FW:
        return px4_ros2::VTOL::State::TransitionToFixedWing;
      case px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_TRANSITION_TO_MC:
        return px4_ros2::VTOL::State::TransitionToMulticopter;
      default:
        return px4_ros2::VTOL::State::Undefined;
    }
  }

  std::shared_ptr<rclcpp::Node> _node;
  rclcpp::executors::SingleThreadedExecutor _executor;
};

// --- VTOLConfig tests ---

TEST(VTOLConfig, DefaultValues)
{
  px4_ros2::VTOLConfig config;
  EXPECT_FLOAT_EQ(config.back_transition_deceleration, 2.f);
  EXPECT_FLOAT_EQ(config.back_transition_deceleration_setpoint_to_pitch_i, 0.1f);
  EXPECT_FLOAT_EQ(config.deceleration_integrator_limit, 0.3f);
  EXPECT_EQ(config.status_timeout, std::chrono::seconds(5));
}

TEST(VTOLConfig, WithStatusTimeout)
{
  auto config = px4_ros2::VTOLConfig{}.withStatusTimeout(std::chrono::seconds(10));
  EXPECT_EQ(config.status_timeout, std::chrono::seconds(10));
}

TEST(VTOLConfig, BuilderChaining)
{
  auto config = px4_ros2::VTOLConfig{}
                    .withBackTransitionDeceleration(3.f)
                    .withStatusTimeout(std::chrono::seconds(8))
                    .withDecelerationIntegratorLimit(0.5f);
  EXPECT_FLOAT_EQ(config.back_transition_deceleration, 3.f);
  EXPECT_FLOAT_EQ(config.deceleration_integrator_limit, 0.5f);
  EXPECT_EQ(config.status_timeout, std::chrono::seconds(8));
}

// --- VTOL transition tests ---

TEST_F(VTOLTest, InitialStateUndefined)
{
  px4_ros2::Context context(*_node);
  px4_ros2::VTOL vtol(context);
  EXPECT_EQ(vtol.getCurrentState(), px4_ros2::VTOL::State::Undefined);
}

TEST_F(VTOLTest, TransitionFailsWithoutStatus)
{
  px4_ros2::Context context(*_node);
  px4_ros2::VTOL vtol(context);

  // No VtolVehicleStatus received yet — transitions should fail
  EXPECT_FALSE(vtol.toMulticopter());
  EXPECT_FALSE(vtol.toFixedwing());
}

TEST_F(VTOLTest, TransitionSucceedsWithFreshStatus)
{
  px4_ros2::Context context(*_node);
  px4_ros2::VTOL vtol(context);

  // Publish a FixedWing status so toMulticopter() has valid state
  ASSERT_TRUE(publishVtolStatus(vtol, px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_FW));

  EXPECT_EQ(vtol.getCurrentState(), px4_ros2::VTOL::State::FixedWing);
  EXPECT_TRUE(vtol.toMulticopter());
}

TEST_F(VTOLTest, ToFixedwingSucceedsFromMulticopter)
{
  px4_ros2::Context context(*_node);
  px4_ros2::VTOL vtol(context);

  ASSERT_TRUE(publishVtolStatus(vtol, px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_MC));

  EXPECT_EQ(vtol.getCurrentState(), px4_ros2::VTOL::State::Multicopter);
  EXPECT_TRUE(vtol.toFixedwing());
}

TEST_F(VTOLTest, StateTransitions)
{
  px4_ros2::Context context(*_node);
  px4_ros2::VTOL vtol(context);

  ASSERT_TRUE(publishVtolStatus(vtol, px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_MC));
  EXPECT_EQ(vtol.getCurrentState(), px4_ros2::VTOL::State::Multicopter);

  ASSERT_TRUE(publishVtolStatus(
      vtol, px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_TRANSITION_TO_FW));
  EXPECT_EQ(vtol.getCurrentState(), px4_ros2::VTOL::State::TransitionToFixedWing);

  ASSERT_TRUE(publishVtolStatus(vtol, px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_FW));
  EXPECT_EQ(vtol.getCurrentState(), px4_ros2::VTOL::State::FixedWing);

  ASSERT_TRUE(publishVtolStatus(
      vtol, px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_TRANSITION_TO_MC));
  EXPECT_EQ(vtol.getCurrentState(), px4_ros2::VTOL::State::TransitionToMulticopter);
}

TEST_F(VTOLTest, CustomTimeoutIsUsed)
{
  // Use a very short timeout (1s) to verify it's configurable
  px4_ros2::Context context(*_node);
  auto config = px4_ros2::VTOLConfig{}.withStatusTimeout(std::chrono::seconds(1));
  px4_ros2::VTOL vtol(context, config);

  ASSERT_TRUE(publishVtolStatus(vtol, px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_FW));
  EXPECT_TRUE(vtol.toMulticopter());

  // Wait for the short timeout to expire
  rclcpp::sleep_for(1100ms);
  EXPECT_FALSE(vtol.toMulticopter());
}
