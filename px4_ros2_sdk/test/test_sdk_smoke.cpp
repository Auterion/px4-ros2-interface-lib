/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <lifecycle_msgs/msg/state.hpp>
#include <memory>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_ros2_sdk/flight_server.hpp>
#include <px4_ros2_sdk/result.hpp>
#include <px4_ros2_sdk/simple_flight.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

using px4_msgs::msg::VehicleCommand;

TEST(SdkResult, ToStringAndCoreMapping)
{
  using px4_ros2::sdk::fromCoreResult;
  using px4_ros2::sdk::Result;
  using px4_ros2::sdk::resultToString;
  EXPECT_STREQ(resultToString(Result::Success), "Success");
  EXPECT_STREQ(resultToString(Result::NotConnected), "NotConnected");
  EXPECT_EQ(fromCoreResult(px4_ros2::Result::Success), Result::Success);
  EXPECT_EQ(fromCoreResult(px4_ros2::Result::Rejected), Result::CommandDenied);
  EXPECT_EQ(fromCoreResult(px4_ros2::Result::Timeout), Result::Timeout);
}

TEST(FlightServer, TakeoffCommandTargetsCurrentPosition)
{
  const auto cmd = px4_ros2::sdk::FlightServer::makeTakeoffCommand(30.f);
  EXPECT_EQ(cmd.command, static_cast<uint32_t>(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF));
  // param5/param6 (lat/lon) must be NaN so Navigator targets the current global
  // position; a finite value (such as a value-initialized 0) commands that
  // literal coordinate instead.
  EXPECT_TRUE(std::isnan(cmd.param5));
  EXPECT_TRUE(std::isnan(cmd.param6));
  EXPECT_TRUE(std::isnan(cmd.param3));
  EXPECT_TRUE(std::isnan(cmd.param4));
  EXPECT_FLOAT_EQ(cmd.param7, 30.f);
  EXPECT_EQ(cmd.target_component, 1);
}

TEST(FlightServer, ArmCommandCarriesOnlyArmStateAndForce)
{
  const auto normal = px4_ros2::sdk::FlightServer::makeArmCommand(/*arm=*/true, /*force=*/false);
  EXPECT_EQ(normal.command,
            static_cast<uint32_t>(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM));
  EXPECT_FLOAT_EQ(normal.param1, 1.f);
  EXPECT_TRUE(std::isnan(normal.param2));  // run preflight checks
  EXPECT_TRUE(std::isnan(normal.param5));

  const auto forced = px4_ros2::sdk::FlightServer::makeArmCommand(/*arm=*/false, /*force=*/true);
  EXPECT_FLOAT_EQ(forced.param1, 0.f);
  EXPECT_FLOAT_EQ(forced.param2, 21196.f);
}

TEST(FlightServer, ConfigureFailsClosedWithoutFmu)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("discovery_timeout_s", 0.3);
  options.append_parameter_override("heartbeat_timeout_s", 0.3);
  auto server = std::make_shared<px4_ros2::sdk::FlightServer>(options);

  // on_configure runs waitForFMU; with no FMU present it fails closed and the
  // node stays unconfigured rather than advancing to inactive.
  const auto& state = server->configure();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(SimpleFlight, ConstructsClients)
{
  auto node = std::make_shared<rclcpp::Node>("simple_flight_test");
  px4_ros2::sdk::SimpleFlight flight(node, "/test_vehicle");
  // Without a running FlightServer, verbs report NotConnected rather than block.
  EXPECT_EQ(flight.arm(false, std::chrono::milliseconds(200)), px4_ros2::sdk::Result::NotConnected);
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
