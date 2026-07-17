/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>

#include <lifecycle_msgs/msg/state.hpp>
#include <memory>
#include <px4_ros2_sdk/flight_server.hpp>
#include <px4_ros2_sdk/result.hpp>
#include <px4_ros2_sdk/simple_flight.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

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
