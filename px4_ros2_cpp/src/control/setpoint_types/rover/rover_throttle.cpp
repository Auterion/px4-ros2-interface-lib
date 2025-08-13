/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/rover/rover_throttle.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

RoverThrottleSetpointType::RoverThrottleSetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _rover_throttle_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::RoverThrottleSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/rover_throttle_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::RoverThrottleSetpoint>(),
    1);
}

void RoverThrottleSetpointType::update(
  const float throttle_body_x,
  const float throttle_body_y)
{
  onUpdate();

  px4_msgs::msg::RoverThrottleSetpoint sp{};
  sp.throttle_body_x = throttle_body_x;
  sp.throttle_body_y = throttle_body_y;
  sp.timestamp = 0; // Let PX4 set the timestamp
  _rover_throttle_setpoint_pub->publish(sp);
}

SetpointBase::Configuration RoverThrottleSetpointType::getConfiguration()
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
