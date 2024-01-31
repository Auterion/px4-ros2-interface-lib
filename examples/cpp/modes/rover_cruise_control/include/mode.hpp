/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_ros2/control/setpoint_types/experimental/manual_rover_control_mode.hpp>
#include <px4_ros2/control/peripheral_actuators.hpp>
// #include <px4_msgs/msg/rover_cruise_control_state.hpp>
// #include <px4_msgs/msg/rover_cruise_control_velocity.hpp>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals; // NOLINT

static const std::string kName = "Rover Cruise Control";
static const std::string kNodeName = "manual_rover_control_mode_node";

#ifndef M_PI_F
#define M_PI_F static_cast<float>(M_PI)
#endif

class ManualRoverControlMode : public px4_ros2::ModeBase
{
public:
  explicit ManualRoverControlMode(rclcpp::Node & node)
  : ModeBase(node, kName)
  {
    _manual_control_input = std::make_shared<px4_ros2::ManualControlInput>(*this);
    _manual_control = std::make_shared<px4_ros2::ManualRoverControlModeType>(*this);
  }

  void onActivate() override
  {
    // _state = _manual_control->initialize_state();
    // _max_linear_velocity_x = _state.parameters[0];
    // _max_angular_velocity_z = _state.parameters[1];
    // _cruise_control_gain = _state.parameters[2];
    // _linear_velocity_x = _state.velocity_state[0];
    // _angular_velocity_z = _state.velocity_state[1];
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    // // Clamp linear velocity to the maximum limits
    // _linear_velocity_x = clampLinearVelocity(
    //   _linear_velocity_x, _max_linear_velocity_x,
    //   _manual_control_input->throttle(), dt_s);

    // Update angular velocity based on roll input
    _linear_velocity_x = _manual_control_input->throttle();
    _angular_velocity_z = _manual_control_input->roll();

    // Update cruise control with the new velocities
    _manual_control->update(_linear_velocity_x, _angular_velocity_z);
  }

  float clampLinearVelocity(float current_velocity, float max_velocity, float throttle, float dt_s)
  {
    if ((current_velocity >= max_velocity && throttle >= 0.0f) ||
      (current_velocity <= -max_velocity && throttle <= 0.0f))
    {
      return copysign(max_velocity, current_velocity);     // Ensures sign consistency
    } else {
      return current_velocity + throttle * _cruise_control_gain * dt_s;
    }
  }

private:
  std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input;
  std::shared_ptr<px4_ros2::ManualRoverControlModeType> _manual_control;

  // px4_msgs::msg::RoverCruiseControlState _state{};

  float _angular_velocity_z{0.f};
  float _linear_velocity_x{0.f};
  float _max_angular_velocity_z{0.f};
  float _max_linear_velocity_x{0.f};
  float _cruise_control_gain{0.f};
};


class manual_rover_control_mode_node : public rclcpp::Node
{
public:
  manual_rover_control_mode_node()
  : Node(kNodeName)
  {
    // Enable debug output
    auto ret =
      rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
    }

    _mode = std::make_unique<ManualRoverControlMode>(*this);

    if (!_mode->doRegister()) {
      throw std::runtime_error("Registration failed");
    }
  }

private:
  std::unique_ptr<ManualRoverControlMode> _mode;
};
