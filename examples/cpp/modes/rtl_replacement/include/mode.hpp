/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/health_and_arming_checks.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/vehicle_state/land_detected.hpp>
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
    _land_detected = std::make_shared<px4_ros2::LandDetected>(*this);
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
    if (_land_detected->landed()) {
      completed(px4_ros2::Result::Success);
      return;
    }

    const Eigen::Vector3f velocity{0.f, 1.f, 5.f};
    _trajectory_setpoint->update(velocity);
  }

private:
  rclcpp::Time _activation_time{};
  std::shared_ptr<px4_ros2::LandDetected> _land_detected;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
};
