/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/experimental/rover/throttle_attitude.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

RoverThrottleAttitudeSetpointType::RoverThrottleAttitudeSetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _rover_throttle_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::RoverThrottleSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/rover_throttle_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::RoverThrottleSetpoint>(),
    1);
  _rover_attitude_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::RoverAttitudeSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/rover_attitude_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::RoverAttitudeSetpoint>(),
    1);
}

void RoverThrottleAttitudeSetpointType::update(
  const float throttle_body_x,
  const float yaw_setpoint,
  std::optional<float> throttle_body_y)
{
  onUpdate();

  px4_msgs::msg::RoverThrottleSetpoint sp_throttle{};
  sp_throttle.throttle_body_x = throttle_body_x;
  sp_throttle.throttle_body_y = throttle_body_y.value_or(NAN);
  sp_throttle.timestamp = 0; // Let PX4 set the timestamp
  _rover_throttle_setpoint_pub->publish(sp_throttle);

  px4_msgs::msg::RoverAttitudeSetpoint sp_att{};
  sp_att.yaw_setpoint = yaw_setpoint;
  sp_att.timestamp = 0; // Let PX4 set the timestamp
  _rover_attitude_setpoint_pub->publish(sp_att);
}

SetpointBase::Configuration RoverThrottleAttitudeSetpointType::getConfiguration()
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
