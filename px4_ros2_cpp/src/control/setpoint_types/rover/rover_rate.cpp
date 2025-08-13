/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/rover/rover_rate.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

RoverRateSetpointType::RoverRateSetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _rover_rate_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::RoverRateSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/rover_rate_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::RoverRateSetpoint>(),
    1);
}

void RoverRateSetpointType::update(
  const float yaw_rate_setpoint)
{
  onUpdate();

  px4_msgs::msg::RoverRateSetpoint sp{};
  sp.yaw_rate_setpoint = yaw_rate_setpoint;
  sp.timestamp = 0; // Let PX4 set the timestamp
  _rover_rate_setpoint_pub->publish(sp);
}

SetpointBase::Configuration RoverRateSetpointType::getConfiguration()
{
  Configuration config{};
  config.control_allocation_enabled = true;
  config.rates_enabled = true;
  config.attitude_enabled = false;
  config.velocity_enabled = false;
  config.position_enabled = false;
  return config;
}
} // namespace px4_ros2
