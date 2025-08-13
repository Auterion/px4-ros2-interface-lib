/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/rover/rover_attitude.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

RoverAttitudeSetpointType::RoverAttitudeSetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _rover_attitude_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::RoverAttitudeSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/rover_attitude_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::RoverAttitudeSetpoint>(),
    1);
}

void RoverAttitudeSetpointType::update(
  const float yaw_setpoint)
{
  onUpdate();

  px4_msgs::msg::RoverAttitudeSetpoint sp{};
  sp.yaw_setpoint = yaw_setpoint;
  sp.timestamp = 0; // Let PX4 set the timestamp
  _rover_attitude_setpoint_pub->publish(sp);
}

SetpointBase::Configuration RoverAttitudeSetpointType::getConfiguration()
{
  Configuration config{};
  config.control_allocation_enabled = true;
  config.rates_enabled = true;
  config.attitude_enabled = true;
  config.velocity_enabled = false;
  config.position_enabled = false;
  return config;
}
} // namespace px4_ros2
