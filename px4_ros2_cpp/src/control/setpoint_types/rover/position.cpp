/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/rover/position.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

RoverPositionSetpointType::RoverPositionSetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _rover_position_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::RoverPositionSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/rover_position_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::RoverPositionSetpoint>(),
    1);
}

void RoverPositionSetpointType::update(
  const Eigen::Vector2f & position_ned,
  const Eigen::Vector2f & start_ned,
  const float cruising_speed,
  const float arrival_speed,
  const float yaw)
{
  onUpdate();

  px4_msgs::msg::RoverPositionSetpoint sp{};
  sp.position_ned[0] = position_ned[0];
  sp.position_ned[1] = position_ned[1];
  sp.start_ned[0] = start_ned[0];
  sp.start_ned[1] = start_ned[1];
  sp.cruising_speed = cruising_speed;
  sp.arrival_speed = arrival_speed;
  sp.yaw = yaw;
  sp.timestamp = 0; // Let PX4 set the timestamp
  _rover_position_setpoint_pub->publish(sp);
}

SetpointBase::Configuration RoverPositionSetpointType::getConfiguration()
{
  Configuration config{};
  config.control_allocation_enabled = true;
  config.rates_enabled = true;
  config.attitude_enabled = true;
  config.velocity_enabled = true;
  config.position_enabled = true;
  return config;
}
} // namespace px4_ros2
