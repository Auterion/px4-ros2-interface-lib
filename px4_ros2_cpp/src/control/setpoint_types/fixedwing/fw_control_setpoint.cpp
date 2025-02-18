/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/fixedwing/fw_control_setpoint.hpp>


namespace px4_ros2
{

    FwControlSetpointType::FwControlSetpointType(Context & context)
	    : SetpointBase(context), _node(context.node())
    {
	_fw_lateral_control_sp_pub=
		context.node().create_publisher<px4_msgs::msg::FwLateralControlSetpoint>(
			context.topicNamespacePrefix() + "fmu/in/fw_lateral_control_setpoint", 1);
    _fw_longitudinal_control_sp_pub =
        context.node().create_publisher<px4_msgs::msg::FwLongitudinalControlSetpoint>(
            context.topicNamespacePrefix() + "fmu/in/fw_longitudinal_control_setpoint", 1);

	_lateral_control_limits_pub = 
		context.node().create_publisher<px4_msgs::msg::LateralControlLimits>(
			context.topicNamespacePrefix() + "fmu/in/lateral_control_limits", 1);
	_longitudinal_control_limits_pub =
		context.node().create_publisher<px4_msgs::msg::LongitudinalControlLimits>(
			context.topicNamespacePrefix() + "fmu/in/longitudinal_control_limits", 1);
    }

    void FwControlSetpointType::update(
			    const float course_setpoint, 
				const float airspeed_reference_direction, 
				const float lateral_acceleration_setpoint, 
				const float altitude_setpoint_msl,
				const float height_rate_setpoint,
				const float equivalent_airspeed_setpoint,
				const float pitch_setpoint,
				const float throttle_setpoint,
				const std::optional<float> & min_pitch,
				const std::optional<float> & max_pitch,
				const std::optional<float> & min_throttle,
				const std::optional<float> & max_throttle,
				const std::optional<float> & min_EAS,
				const std::optional<float> & max_EAS,
				const std::optional<float> & max_lat_acc,
				const std::optional<float> & target_climb_rate,
				const std::optional<float> & target_sink_rate)
    {
	onUpdate();

	px4_msgs::msg::FwLateralControlSetpoint lat_sp{};
	lat_sp.course_setpoint = course_setpoint;
	lat_sp.airspeed_reference_direction = airspeed_reference_direction;
	lat_sp.lateral_acceleration_setpoint = lateral_acceleration_setpoint;

	lat_sp.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
	_fw_lateral_control_sp_pub->publish(lat_sp);

    px4_msgs::msg::FwLongitudinalControlSetpoint lon_sp{};
    lon_sp.altitude_setpoint = altitude_setpoint_msl;
    lon_sp.height_rate_setpoint = height_rate_setpoint;
    lon_sp.equivalent_airspeed_setpoint = equivalent_airspeed_setpoint;
    lon_sp.pitch_sp = pitch_setpoint;
    lon_sp.thrust_sp = throttle_setpoint;

    lon_sp.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
    _fw_longitudinal_control_sp_pub->publish(lon_sp);

	px4_msgs::msg::LateralControlLimits lat_limits{};
	bool publish_lat_limits =  !std::isnan(max_lat_acc.value());
	
	if (publish_lat_limits) {
		lat_limits.lateral_accel_max = *max_lat_acc;

		lat_limits.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
		_lateral_control_limits_pub->publish(lat_limits);
	}

	px4_msgs::msg::LongitudinalControlLimits lon_limits{};

	bool publish_lon_limits = !std::isnan(min_pitch.value()) && !std::isnan(max_pitch.value()) &&
		!std::isnan(min_throttle.value()) && !std::isnan(max_throttle.value()) &&
		!std::isnan(min_EAS.value()) && !std::isnan(max_EAS.value()) &&
		!std::isnan(target_climb_rate.value()) && !std::isnan(target_sink_rate.value());
	
	bool incomplete_lon_limits = !publish_lon_limits && 
		(!std::isnan(min_pitch.value()) || !std::isnan(max_pitch.value()) ||
		!std::isnan(min_throttle.value()) || !std::isnan(max_throttle.value()) ||
		!std::isnan(min_EAS.value()) || !std::isnan(max_EAS.value()) ||
		!std::isnan(target_climb_rate.value()) || !std::isnan(target_sink_rate.value()));

	if (publish_lon_limits) {
		lon_limits.pitch_min = *min_pitch;
		lon_limits.pitch_max = *max_pitch;
		lon_limits.throttle_min = *min_throttle;
		lon_limits.throttle_max = *max_throttle;
		lon_limits.equivalent_airspeed_min = *min_EAS;
		lon_limits.equivalent_airspeed_max = *max_EAS;
		lon_limits.climb_rate_target = *target_climb_rate;
		lon_limits.sink_rate_target = *target_sink_rate;

		lon_limits.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
		_longitudinal_control_limits_pub->publish(lon_limits);
	} else if (incomplete_lon_limits) {
		// Log warning if incomplete limits are set
		RCLCPP_WARN_ONCE(_node.get_logger(), "Incomplete longitudinal control limits set. Using default PX4 limits.");
	}
	}

    SetpointBase::Configuration FwControlSetpointType::getConfiguration()
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
