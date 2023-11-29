/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/experimental/rover_cruise_control.hpp>


namespace px4_ros2
{

CruiseControlSetpointType::CruiseControlSetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _differential_drive_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::DifferentialDriveSetpoint>(
    context.topicNamespacePrefix() + "/fmu/in/differential_drive_setpoint", 1);

  _rover_cruise_control =
    context.node().create_subscription<px4_msgs::msg::RoverCruiseControlState>(
    "/fmu/out/rover_cruise_control_state", rclcpp::QoS(
      1).best_effort(),
    [this](const px4_msgs::msg::RoverCruiseControlState::SharedPtr msg) {
      _max_linear_velocity_x = msg->parameters[0];
      _max_angular_velocity_z = msg->parameters[1];
      _cruise_control_gain = msg->parameters[2];
      _linear_velocity_x = msg->velocity_state[0];
      _angular_velocity_z = msg->velocity_state[1];
    });
}

px4_msgs::msg::RoverCruiseControlState CruiseControlSetpointType::initialize_state()
{
  px4_msgs::msg::RoverCruiseControlState state{};

  state.parameters[0] = _max_linear_velocity_x;
  state.parameters[1] = _max_angular_velocity_z;
  state.parameters[2] = _cruise_control_gain;
  state.velocity_state[0] = _linear_velocity_x;
  state.velocity_state[1] = _angular_velocity_z;

  return state;
}

void CruiseControlSetpointType::update(float linear_velocity_x, float angular_velocity_z)
{
  onUpdate();

  px4_msgs::msg::DifferentialDriveSetpoint vel{};

  vel.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
  vel.velocity = linear_velocity_x;
  vel.yaw = angular_velocity_z;

  _differential_drive_setpoint_pub->publish(vel);
}

SetpointBase::Configuration CruiseControlSetpointType::getConfiguration()
{
  Configuration config{};
  config.attitude_enabled = false;
  config.altitude_enabled = false;
  config.climb_rate_enabled = false;
  config.acceleration_enabled = false;
  config.velocity_enabled = false;
  config.position_enabled = false;

  return config;
}
} // namespace px4_ros2
