/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_sdk/components/mode.hpp>
#include <px4_sdk/components/mode_executor.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

using namespace std::chrono_literals; // NOLINT

static const char * name = "Custom RTL";
static const char * node_name = "example_mode_rtl";

class FlightModeTest : public px4_sdk::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, Settings{name, true, ModeBase::kModeIDRtl},
      px4_sdk::ModeRequirements::autonomous())
  {
    _vehicle_land_detected_sub = node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", rclcpp::QoS(1).best_effort(),
      [this](px4_msgs::msg::VehicleLandDetected::UniquePtr msg) {
        _landed = msg->landed;
      });
    setSetpointUpdateRate(30.F);
  }

  void onActivate() override
  {
    _activation_time = node().get_clock()->now();
    setpoints().configureSetpointsSync(px4_sdk::SetpointSender::SetpointConfiguration{});
  }

  void onDeactivate() override {}

  void updateSetpoint() override
  {
    if (_landed) {
      completed(px4_sdk::Result::Success);
      return;
    }

    const Eigen::Vector3f velocity{0.F, 1.F, 5.F};
    setpoints().sendTrajectorySetpoint(velocity);
  }

private:
  rclcpp::Time _activation_time{};
  bool _landed{true};
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;
};

class TestNode : public rclcpp::Node
{
public:
  TestNode()
  : Node(node_name)
  {
    // Enable debug output
    auto ret =
      rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
    }

    _mode = std::make_unique<FlightModeTest>(*this);

    if (!_mode->doRegister()) {
      throw std::runtime_error("Registration failed");
    }
  }

private:
  std::unique_ptr<FlightModeTest> _mode;
};
