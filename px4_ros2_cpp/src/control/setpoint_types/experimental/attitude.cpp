/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>


namespace px4_ros2
{

AttitudeSetpointType::AttitudeSetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _vehicle_attitude_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
    context.topicNamespacePrefix() + "/fmu/in/vehicle_attitude_setpoint", 1);
}

void AttitudeSetpointType::update(
  const Eigen::Quaternionf & attitude_setpoint,
  const Eigen::Vector3f & thrust_setpoint_frd,
  float yaw_sp_move_rate_rad_s)
{
  onUpdate();

  px4_msgs::msg::VehicleAttitudeSetpoint sp{};
  sp.q_d[0] = attitude_setpoint.w();
  sp.q_d[1] = attitude_setpoint.x();
  sp.q_d[2] = attitude_setpoint.y();
  sp.q_d[3] = attitude_setpoint.z();
  sp.thrust_body[0] = thrust_setpoint_frd(0);
  sp.thrust_body[1] = thrust_setpoint_frd(1);
  sp.thrust_body[2] = thrust_setpoint_frd(2);
  sp.yaw_sp_move_rate = yaw_sp_move_rate_rad_s;
  sp.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
  _vehicle_attitude_setpoint_pub->publish(sp);
}

SetpointBase::Configuration AttitudeSetpointType::getConfiguration()
{
  Configuration config{};
  config.altitude_enabled = false;
  config.climb_rate_enabled = false;
  config.acceleration_enabled = false;
  config.velocity_enabled = false;
  config.position_enabled = false;
  return config;
}
} // namespace px4_ros2
