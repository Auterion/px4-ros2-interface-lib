/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_sdk/components/mode.hpp>
#include <px4_sdk/components/health_and_arming_checks.hpp>
#include <px4_sdk/components/mode_executor.hpp>
#include <px4_sdk/control/setpoint_types/trajectory.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

using namespace std::chrono_literals; // NOLINT

static const std::string kName = "Custom RTL";
static const std::string kNodeName = "example_mode_rtl";

class FlightModeTest : public px4_sdk::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, Settings{kName, true, ModeBase::kModeIDRtl})
  {
    _vehicle_land_detected_sub = node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", rclcpp::QoS(1).best_effort(),
      [this](px4_msgs::msg::VehicleLandDetected::UniquePtr msg) {
        _landed = msg->landed;
      });
    _trajectory_setpoint = std::make_shared<px4_sdk::TrajectorySetpointType>(*this);

    modeRequirements().home_position = true;
  }

  void onActivate() override
  {
    _activation_time = node().get_clock()->now();
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    if (_landed) {
      completed(px4_sdk::Result::Success);
      return;
    }

    const Eigen::Vector3f velocity{0.F, 1.F, 5.F};
    _trajectory_setpoint->update(velocity);
  }

private:
  rclcpp::Time _activation_time{};
  bool _landed{true};
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;
  std::shared_ptr<px4_sdk::TrajectorySetpointType> _trajectory_setpoint;
};

class TestNode : public rclcpp::Node
{
public:
  TestNode()
  : Node(kNodeName)
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
