/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/experimental/rover/speed_rate.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

RoverSpeedRateSetpointType::RoverSpeedRateSetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _rover_speed_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::RoverSpeedSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/rover_speed_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::RoverSpeedSetpoint>(),
    1);
  _rover_rate_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::RoverRateSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/rover_rate_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::RoverRateSetpoint>(),
    1);
}

void RoverSpeedRateSetpointType::update(
  const float speed_body_x,
  const float yaw_rate_setpoint,
  std::optional<float> speed_body_y)
{
  onUpdate();

  px4_msgs::msg::RoverSpeedSetpoint sp_speed{};
  sp_speed.speed_body_x = speed_body_x;
  sp_speed.speed_body_y = speed_body_y.value_or(NAN);
  sp_speed.timestamp = 0; // Let PX4 set the timestamp
  _rover_speed_setpoint_pub->publish(sp_speed);

  px4_msgs::msg::RoverRateSetpoint sp_rate{};
  sp_rate.yaw_rate_setpoint = yaw_rate_setpoint;
  sp_rate.timestamp = 0; // Let PX4 set the timestamp
  _rover_rate_setpoint_pub->publish(sp_rate);
}

SetpointBase::Configuration RoverSpeedRateSetpointType::getConfiguration()
{
  Configuration config{};
  config.control_allocation_enabled = true;
  config.rates_enabled = true;
  config.attitude_enabled = false;
  config.velocity_enabled = true;
  config.position_enabled = false;
  return config;
}
} // namespace px4_ros2
