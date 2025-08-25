/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rover/speed_rate.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <rclcpp/rclcpp.hpp>


static const std::string kName = "Rover Velocity Mode";
static const float kMaxSpeed = 2.5f; // [m/s] Set equal to RO_SPEED_LIM
static const float kMaxYawRate = 130.0f; // [deg/s] Set equal to RO_YAW_RATE_LIM

class RoverVelocityMode : public px4_ros2::ModeBase
{
public:
  explicit RoverVelocityMode(rclcpp::Node & node)
  : ModeBase(node, kName)
  {
    _manual_control_input = std::make_shared<px4_ros2::ManualControlInput>(*this);
    _rover_speed_rate_setpoint = std::make_shared<px4_ros2::RoverSpeedRateSetpointType>(*this);
  }

  void onActivate() override {}

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    const float speed_body_x = _manual_control_input->throttle() * kMaxSpeed;
    const float yaw_rate = _manual_control_input->roll() * px4_ros2::degToRad(kMaxYawRate);
    _rover_speed_rate_setpoint->update(speed_body_x, yaw_rate);
  }

private:
  std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input;
  std::shared_ptr<px4_ros2::RoverSpeedRateSetpointType> _rover_speed_rate_setpoint;
};
