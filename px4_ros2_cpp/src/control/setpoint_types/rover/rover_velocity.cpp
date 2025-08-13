/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/rover/rover_velocity.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

RoverVelocitySetpointType::RoverVelocitySetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _rover_velocity_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::RoverVelocitySetpoint>(
    context.topicNamespacePrefix() + "fmu/in/rover_velocity_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::RoverVelocitySetpoint>(),
    1);
}

void RoverVelocitySetpointType::update(
  const float speed,
  const float bearing,
  const float yaw)
{
  onUpdate();

  px4_msgs::msg::RoverVelocitySetpoint sp{};
  sp.speed = speed;
  sp.bearing = bearing;
  sp.yaw = yaw;
  sp.timestamp = 0; // Let PX4 set the timestamp
  _rover_velocity_setpoint_pub->publish(sp);
}

SetpointBase::Configuration RoverVelocitySetpointType::getConfiguration()
{
  Configuration config{};
  config.control_allocation_enabled = true;
  config.rates_enabled = true;
  config.attitude_enabled = true;
  config.velocity_enabled = true;
  config.position_enabled = false;
  return config;
}
} // namespace px4_ros2
