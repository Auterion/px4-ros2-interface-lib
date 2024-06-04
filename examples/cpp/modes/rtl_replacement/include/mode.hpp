/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/health_and_arming_checks.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

using namespace std::chrono_literals; // NOLINT

static const std::string kName = "Custom RTL";

class FlightModeTest : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, Settings{kName, true, ModeBase::kModeIDRtl})
  {
    _vehicle_land_detected_sub = node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "fmu/out/vehicle_land_detected", rclcpp::QoS(1).best_effort(),
      [this](px4_msgs::msg::VehicleLandDetected::UniquePtr msg) {
        _landed = msg->landed;
      });
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

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
      completed(px4_ros2::Result::Success);
      return;
    }

    const Eigen::Vector3f velocity{0.f, 1.f, 5.f};
    _trajectory_setpoint->update(velocity);
  }

private:
  rclcpp::Time _activation_time{};
  bool _landed{true};
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
};
