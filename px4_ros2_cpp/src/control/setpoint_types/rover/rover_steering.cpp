/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/rover/rover_steering.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

RoverSteeringSetpointType::RoverSteeringSetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _rover_steering_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::RoverSteeringSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/rover_steering_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::RoverSteeringSetpoint>(),
    1);
}

void RoverSteeringSetpointType::update(
  const float normalized_steering_setpoint)
{
  onUpdate();

  px4_msgs::msg::RoverSteeringSetpoint sp{};
  sp.normalized_steering_setpoint = normalized_steering_setpoint;
  sp.timestamp = 0; // Let PX4 set the timestamp
  _rover_steering_setpoint_pub->publish(sp);
}

SetpointBase::Configuration RoverSteeringSetpointType::getConfiguration()
{
  Configuration config{};
  config.control_allocation_enabled = true;
  config.rates_enabled = false;
  config.attitude_enabled = false;
  config.velocity_enabled = false;
  config.position_enabled = false;
  return config;
}
} // namespace px4_ros2
