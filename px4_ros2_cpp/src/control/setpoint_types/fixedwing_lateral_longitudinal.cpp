/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/fixedwing_lateral_longitudinal.hpp>
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

  _lateral_control_configuration_pub =
    context.node().create_publisher<px4_msgs::msg::LateralControlConfiguration>(
    context.topicNamespacePrefix() + "fmu/in/lateral_control_configuration" + px4_ros2::getMessageNameVersion<px4_msgs::msg::LateralControlConfiguration>(),
    1);
  _longitudinal_control_configuration_pub =
    context.node().create_publisher<px4_msgs::msg::LongitudinalControlConfiguration>(
    context.topicNamespacePrefix() + "fmu/in/longitudinal_control_configuration" + px4_ros2::getMessageNameVersion<px4_msgs::msg::LongitudinalControlConfiguration>(),
    1);
}

void FwLateralLongitudinalSetpointType::updateWithAltitude(
  const float altitude_amsl_sp, const float course_sp,
  std::optional<float> equivalent_airspeed_sp,
  std::optional<float> lateral_acceleration_sp)
{
  onUpdate();

  px4_msgs::msg::FixedWingLateralSetpoint lateral_sp{};
  lateral_sp.course = course_sp;
  lateral_sp.airspeed_direction = NAN;
  lateral_sp.lateral_acceleration = lateral_acceleration_sp.value_or(NAN);

  _fw_lateral_sp_pub->publish(lateral_sp);

  px4_msgs::msg::FixedWingLongitudinalSetpoint longitudinal_sp{};
  longitudinal_sp.altitude = altitude_amsl_sp;
  longitudinal_sp.height_rate = NAN;
  longitudinal_sp.equivalent_airspeed = equivalent_airspeed_sp.value_or(NAN);
  longitudinal_sp.pitch_direct = NAN;
  longitudinal_sp.throttle_direct = NAN;

  _fw_longitudinal_sp_pub->publish(longitudinal_sp);

}

void FwLateralLongitudinalSetpointType::updateWithHeightRate(
  const float height_rate_sp, const float course_sp,
  std::optional<float> equivalent_airspeed_sp,
  std::optional<float> lateral_acceleration_sp)
{
  onUpdate();

  px4_msgs::msg::FixedWingLateralSetpoint lateral_sp{};
  lateral_sp.course = course_sp;
  lateral_sp.airspeed_direction = NAN;
  lateral_sp.lateral_acceleration = lateral_acceleration_sp.value_or(NAN);

  _fw_lateral_sp_pub->publish(lateral_sp);

  px4_msgs::msg::FixedWingLongitudinalSetpoint longitudinal_sp{};
  longitudinal_sp.altitude = NAN;
  longitudinal_sp.height_rate = height_rate_sp;
  longitudinal_sp.equivalent_airspeed = equivalent_airspeed_sp.value_or(NAN);
  longitudinal_sp.pitch_direct = NAN;
  longitudinal_sp.throttle_direct = NAN;

  _fw_longitudinal_sp_pub->publish(longitudinal_sp);
}

void FwLateralLongitudinalSetpointType::update(
  const FwLateralLongitudinalSetpoint & setpoint,
  const FwControlConfiguration & config)
{
  onUpdate();

  update(setpoint);

  px4_msgs::msg::LateralControlConfiguration lateral_configuration{};
  lateral_configuration.lateral_accel_max = config.max_lateral_acceleration.value_or(NAN);

  _lateral_control_configuration_pub->publish(lateral_configuration);

  px4_msgs::msg::LongitudinalControlConfiguration longitudinal_configuration{};
  longitudinal_configuration.pitch_min = config.min_pitch.value_or(NAN);
  longitudinal_configuration.pitch_max = config.max_pitch.value_or(NAN);
  longitudinal_configuration.throttle_min = config.min_throttle.value_or(NAN);
  longitudinal_configuration.throttle_max = config.max_throttle.value_or(NAN);
  longitudinal_configuration.climb_rate_target = config.target_climb_rate.value_or(NAN);
  longitudinal_configuration.sink_rate_target = config.target_sink_rate.value_or(NAN);

  _longitudinal_control_configuration_pub->publish(longitudinal_configuration);

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
