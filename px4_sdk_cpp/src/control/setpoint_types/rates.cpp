/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_sdk/control/setpoint_types/rates.hpp>


namespace px4_sdk
{

RatesSetpointType::RatesSetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _vehicle_rates_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
    context.topicNamespacePrefix() + "/fmu/in/vehicle_rates_setpoint", 1);
}

void RatesSetpointType::update(
  const Eigen::Vector3f & rate_setpoints_ned_rad,
  const Eigen::Vector3f & thrust_setpoint_ned)
{
  onUpdate();

  px4_msgs::msg::VehicleRatesSetpoint sp{};
  sp.roll = rate_setpoints_ned_rad(0);
  sp.pitch = rate_setpoints_ned_rad(1);
  sp.yaw = rate_setpoints_ned_rad(2);
  sp.thrust_body[0] = thrust_setpoint_ned(0);
  sp.thrust_body[1] = thrust_setpoint_ned(1);
  sp.thrust_body[2] = thrust_setpoint_ned(2);
  sp.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
  _vehicle_rates_setpoint_pub->publish(sp);
}

SetpointBase::Configuration RatesSetpointType::getConfiguration()
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
} // namespace px4_sdk
