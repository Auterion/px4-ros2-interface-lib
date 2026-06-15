/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/fixedwing/global_path_setpoint.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

FwGlobalPathSetpointType::FwGlobalPathSetpointType(Context & context)
: SetpointBase(context)
{
  _global_path_sp_pub =
    context.node().create_publisher<px4_msgs::msg::GlobalPathSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/global_path_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::GlobalPathSetpoint>(),
    1);
}

void FwGlobalPathSetpointType::update(const FwGlobalPathSetpoint & setpoint)
{
  onUpdate();

  px4_msgs::msg::GlobalPathSetpoint sp{};
  sp.timestamp = 0;  // let PX4 set the timestamp
  sp.lat = setpoint.lat.value_or(std::numeric_limits<double>::quiet_NaN());
  sp.lon = setpoint.lon.value_or(std::numeric_limits<double>::quiet_NaN());
  sp.alt = setpoint.alt.value_or(NAN);

  if (setpoint.tangent.has_value()) {
    sp.tangent[0] = (*setpoint.tangent)(0);
    sp.tangent[1] = (*setpoint.tangent)(1);
    sp.tangent[2] = (*setpoint.tangent)(2);
  } else {
    sp.tangent[0] = NAN;
    sp.tangent[1] = NAN;
    sp.tangent[2] = NAN;
  }

  sp.height_rate = setpoint.height_rate.value_or(NAN);
  sp.curvature = setpoint.curvature.value_or(NAN);

  _global_path_sp_pub->publish(sp);
}

SetpointBase::Configuration FwGlobalPathSetpointType::getConfiguration()
{
  Configuration config{};
  config.control_allocation_enabled = true;
  config.rates_enabled = true;
  config.attitude_enabled = true;
  config.altitude_enabled = true;
  config.acceleration_enabled = true;
  config.velocity_enabled = true;
  config.position_enabled = true;
  config.climb_rate_enabled = true;
  return config;
}

} // namespace px4_ros2
