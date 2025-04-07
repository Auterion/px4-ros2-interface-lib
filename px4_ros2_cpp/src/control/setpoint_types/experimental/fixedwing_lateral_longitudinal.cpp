/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/experimental/fixedwing_lateral_longitudinal.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

FwLateralLongitudinalSetpointType::FwLateralLongitudinalSetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _fw_lateral_sp_pub =
    context.node().create_publisher<px4_msgs::msg::FixedWingLateralSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/fixed_wing_lateral_setpoint" + px4_ros2::getMessageNameVersion<px4_msgs::msg::FixedWingLateralSetpoint>(),
    1);
  _fw_longitudinal_sp_pub =
    context.node().create_publisher<px4_msgs::msg::FixedWingLongitudinalSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/fixed_wing_longitudinal_setpoint" + px4_ros2::getMessageNameVersion<px4_msgs::msg::FixedWingLongitudinalSetpoint>(),
    1);

  _lateral_control_limits_pub =
    context.node().create_publisher<px4_msgs::msg::LateralControlLimits>(
    context.topicNamespacePrefix() + "fmu/in/lateral_control_limits" + px4_ros2::getMessageNameVersion<px4_msgs::msg::LateralControlLimits>(),
    1);
  _longitudinal_control_limits_pub =
    context.node().create_publisher<px4_msgs::msg::LongitudinalControlLimits>(
    context.topicNamespacePrefix() + "fmu/in/longitudinal_control_limits" + px4_ros2::getMessageNameVersion<px4_msgs::msg::LongitudinalControlLimits>(),
    1);
}

void FwLateralLongitudinalSetpointType::update(
  const FwLateralLongitudinalSetpoint & setpoint,
  const FwControlLimits & limits)
{
  onUpdate();

  update(setpoint);

  px4_msgs::msg::LateralControlLimits lateral_limits{};
  lateral_limits.lateral_accel_max = limits.max_lateral_acceleration.value_or(NAN);

  _lateral_control_limits_pub->publish(lateral_limits);

  px4_msgs::msg::LongitudinalControlLimits longitudinal_limits{};
  longitudinal_limits.pitch_min = limits.min_pitch.value_or(NAN);
  longitudinal_limits.pitch_max = limits.max_pitch.value_or(NAN);
  longitudinal_limits.throttle_min = limits.min_throttle.value_or(NAN);
  longitudinal_limits.throttle_max = limits.max_throttle.value_or(NAN);
  longitudinal_limits.equivalent_airspeed_min = limits.min_equivalent_airspeed.value_or(NAN);
  longitudinal_limits.equivalent_airspeed_max = limits.max_equivalent_airspeed.value_or(NAN);
  longitudinal_limits.climb_rate_target = limits.target_climb_rate.value_or(NAN);
  longitudinal_limits.sink_rate_target = limits.target_sink_rate.value_or(NAN);

  _longitudinal_control_limits_pub->publish(longitudinal_limits);

}

void FwLateralLongitudinalSetpointType::update(const FwLateralLongitudinalSetpoint & setpoint)
{
  onUpdate();

  px4_msgs::msg::FixedWingLateralSetpoint lateral_sp{};
  lateral_sp.course = setpoint.course.value_or(NAN);
  lateral_sp.airspeed_direction = setpoint.airspeed_direction.value_or(NAN);
  lateral_sp.lateral_acceleration = setpoint.lateral_acceleration.value_or(NAN);

  _fw_lateral_sp_pub->publish(lateral_sp);

  px4_msgs::msg::FixedWingLongitudinalSetpoint longitudinal_sp{};
  longitudinal_sp.altitude = setpoint.altitude_msl.value_or(NAN);
  longitudinal_sp.height_rate = setpoint.height_rate.value_or(NAN);
  longitudinal_sp.equivalent_airspeed = setpoint.equivalent_airspeed.value_or(NAN);
  longitudinal_sp.pitch_direct = NAN;
  longitudinal_sp.throttle_direct = NAN;

  _fw_longitudinal_sp_pub->publish(longitudinal_sp);
}

SetpointBase::Configuration FwLateralLongitudinalSetpointType::getConfiguration()
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
