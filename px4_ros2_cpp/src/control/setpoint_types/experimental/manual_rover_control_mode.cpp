/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/experimental/manual_rover_control_mode.hpp>


namespace px4_ros2
{

ManualRoverControlModeType::ManualRoverControlModeType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _differential_drive_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::DifferentialDriveSetpoint>(
    context.topicNamespacePrefix() + "/fmu/in/differential_drive_setpoint", 1);

}

void ManualRoverControlModeType::update(float speed, float yaw_rate)
{
  onUpdate();

  px4_msgs::msg::DifferentialDriveSetpoint setpoint{};

  setpoint.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
  setpoint.speed = speed;
  setpoint.yaw_rate = yaw_rate;
  setpoint.closed_loop_speed_control = false; // false for now, we can do closed loop control with wheel odom
  setpoint.closed_loop_yaw_rate_control = false; // same case here


  _differential_drive_setpoint_pub->publish(setpoint);
}

SetpointBase::Configuration ManualRoverControlModeType::getConfiguration()
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
