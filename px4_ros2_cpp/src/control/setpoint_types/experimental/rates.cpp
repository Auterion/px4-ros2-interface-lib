/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

RatesSetpointType::RatesSetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _vehicle_rates_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/vehicle_rates_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleRatesSetpoint>(),
    1);
}

void RatesSetpointType::update(
  const Eigen::Vector3f & rate_setpoints_ned_rad,
  const Eigen::Vector3f & thrust_setpoint_frd)
{
  onUpdate();

  px4_msgs::msg::VehicleRatesSetpoint sp{};
  sp.roll = rate_setpoints_ned_rad(0);
  sp.pitch = rate_setpoints_ned_rad(1);
  sp.yaw = rate_setpoints_ned_rad(2);
  sp.thrust_body[0] = thrust_setpoint_frd(0);
  sp.thrust_body[1] = thrust_setpoint_frd(1);
  sp.thrust_body[2] = thrust_setpoint_frd(2);
  sp.timestamp = 0; // Let PX4 set the timestamp
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
} // namespace px4_ros2
