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
	_fw_lateral_sp_pub=
		context.node().create_publisher<px4_msgs::msg::FixedWingLateralSetpoint>(
			context.topicNamespacePrefix() + "fmu/in/fixed_wing_lateral_setpoint" + px4_ros2::getMessageNameVersion<px4_msgs::msg::FixedWingLateralSetpoint>(), 1);
    _fw_longitudinal_sp_pub =
        context.node().create_publisher<px4_msgs::msg::FixedWingLongitudinalSetpoint>(
            context.topicNamespacePrefix() + "fmu/in/fixed_wing_longitudinal_setpoint" + px4_ros2::getMessageNameVersion<px4_msgs::msg::FixedWingLongitudinalSetpoint>(), 1);

	_lateral_control_limits_pub = 
		context.node().create_publisher<px4_msgs::msg::LateralControlLimits>(
			context.topicNamespacePrefix() + "fmu/in/lateral_control_limits" + px4_ros2::getMessageNameVersion<px4_msgs::msg::LateralControlLimits>(), 1);
	_longitudinal_control_limits_pub =
		context.node().create_publisher<px4_msgs::msg::LongitudinalControlLimits>(
			context.topicNamespacePrefix() + "fmu/in/longitudinal_control_limits"+ px4_ros2::getMessageNameVersion<px4_msgs::msg::LongitudinalControlLimits>(), 1);
    }

    void FwLateralLongitudinalSetpointType::update(
			const float course_setpoint, 
			const float airspeed_direction, 
			const float lateral_acceleration_setpoint, 
			const float altitude_setpoint_msl,
			const float height_rate_setpoint,
			const float equivalent_airspeed_setpoint,
			const std::optional<float> & min_pitch,
			const std::optional<float> & max_pitch,
			const std::optional<float> & min_throttle,
			const std::optional<float> & max_throttle,
			const std::optional<float> & min_equivalent_airspeed,
			const std::optional<float> & max_equivalent_airspeed,
			const std::optional<float> & max_lat_acc,
			const std::optional<float> & target_climb_rate,
			const std::optional<float> & target_sink_rate)
    {
	onUpdate();

	px4_msgs::msg::FixedWingLateralSetpoint lateral_sp{};
	lateral_sp.course = course_setpoint;
	lateral_sp.airspeed_direction = airspeed_direction;
	lateral_sp.lateral_acceleration = lateral_acceleration_setpoint;

	lateral_sp.timestamp = 0; // Let PX4 set the timestamp
	_fw_lateral_sp_pub->publish(lateral_sp);

    px4_msgs::msg::FixedWingLongitudinalSetpoint longitudinal_sp{};
    longitudinal_sp.altitude = altitude_setpoint_msl;
    longitudinal_sp.height_rate = height_rate_setpoint;
    longitudinal_sp.equivalent_airspeed = equivalent_airspeed_setpoint;
    longitudinal_sp.pitch_direct = NAN; 
    longitudinal_sp.throttle_direct = NAN; 

    longitudinal_sp.timestamp = 0; // Let PX4 set the timestamp
    _fw_longitudinal_sp_pub->publish(longitudinal_sp);

	px4_msgs::msg::LateralControlLimits lateral_limits{};
	lateral_limits.lateral_accel_max = *max_lat_acc;

	lateral_limits.timestamp = 0; // Let PX4 set the timestamp
	_lateral_control_limits_pub->publish(lateral_limits);

	px4_msgs::msg::LongitudinalControlLimits longitudinal_limits{};
	longitudinal_limits.pitch_min = *min_pitch;
	longitudinal_limits.pitch_max = *max_pitch;
	longitudinal_limits.throttle_min = *min_throttle;
	longitudinal_limits.throttle_max = *max_throttle;
	longitudinal_limits.equivalent_airspeed_min = *min_equivalent_airspeed;
	longitudinal_limits.equivalent_airspeed_max = *max_equivalent_airspeed;
	longitudinal_limits.climb_rate_target = *target_climb_rate;
	longitudinal_limits.sink_rate_target = *target_sink_rate;

	longitudinal_limits.timestamp = 0; // Let PX4 set the timestamp
	_longitudinal_control_limits_pub->publish(longitudinal_limits);
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