/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/rover/rover_velocity.hpp>
#include <px4_ros2/control/setpoint_types/rover/rover_rate.hpp>
#include <px4_msgs/msg/rover_rate_setpoint.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals; // NOLINT

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
    _rover_velocity_setpoint = std::make_shared<px4_ros2::RoverVelocitySetpointType>(*this);
    _rover_rate_setpoint_pub = node.create_publisher<px4_msgs::msg::RoverRateSetpoint>(
      "fmu/in/rover_rate_setpoint", 1);
  }

  void onActivate() override {}

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    const float speed = _manual_control_input->throttle() * kMaxSpeed;
    _rover_velocity_setpoint->update(speed, NAN, NAN);

    // Rate is not updated through the setpoint type to avoid overriding vehicle control mode.
    const float yaw_rate = _manual_control_input->roll() * kMaxYawRate * M_PI / 180.f;
    px4_msgs::msg::RoverRateSetpoint sp{};
    sp.yaw_rate_setpoint = yaw_rate;
    sp.timestamp = 0; // Let PX4 set the timestamp
    _rover_rate_setpoint_pub->publish(sp);

  }

private:
  std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input;
  std::shared_ptr<px4_ros2::RoverVelocitySetpointType> _rover_velocity_setpoint;
  rclcpp::Publisher<px4_msgs::msg::RoverRateSetpoint>::SharedPtr _rover_rate_setpoint_pub;
};
