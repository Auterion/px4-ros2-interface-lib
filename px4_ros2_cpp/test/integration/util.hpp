/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

std::shared_ptr<rclcpp::Node> initNode();

class BaseTest : public ::testing::Test
{
public:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

class ModesTest : public BaseTest
{
};

class VehicleState
{
public:
  explicit VehicleState(rclcpp::Node & node, const std::string & topic_namespace_prefix = "");

  using VehicleStatusUpdateCallback =
    std::function<void (const px4_msgs::msg::VehicleStatus::UniquePtr &)>;

  using ModeSetCallback = std::function<void ()>;

  const VehicleStatusUpdateCallback & getOnVehicleStatusUpdate() const
  {return _on_vehicle_status_update;}

  void setOnVehicleStatusUpdate(const VehicleStatusUpdateCallback & on_vehicle_status_update)
  {_on_vehicle_status_update = on_vehicle_status_update;}

  void callbackOnModeSet(const ModeSetCallback & callback, uint8_t nav_state);

  void sendCommand(
    uint32_t command, float param1 = NAN, float param2 = NAN, float param3 = NAN,
    float param4 = NAN,
    float param5 = NAN, float param6 = NAN, float param7 = NAN);

  // Methods to trigger failsafes
  void setGPSFailure(bool failure);
  void setForceLowBattery(bool enabled);

private:
  rclcpp::Node & _node;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
  VehicleStatusUpdateCallback _on_vehicle_status_update;
  ModeSetCallback _on_mode_set_callback;
  uint8_t _waiting_for_nav_state{};
  int _matching_nav_state_set{0};
};
