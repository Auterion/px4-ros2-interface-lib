/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/fixedwing/fw_control_setpoint.hpp>
#include <px4_msgs/msg/fw_longitudinal_control_setpoint.hpp>
#include <px4_msgs/msg/fw_lateral_control_setpoint.hpp>


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
    }

    void FwControlSetpointType::update(
	    const float course_setpoint, const float heading_setpoint, const float lateral_acceleration_setpoint, const float altitude_setpoint_msl)
    {
	onUpdate();

	px4_msgs::msg::FwLateralControlSetpoint lat_sp{};
	lat_sp.course_setpoint = course_setpoint;
	lat_sp.airspeed_reference_direction = heading_setpoint;
	lat_sp.lateral_acceleration_setpoint = lateral_acceleration_setpoint;

	lat_sp.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
	_fw_lateral_control_sp_pub->publish(lat_sp);

    px4_msgs::msg::FwLongitudinalControlSetpoint lon_sp{};
    lon_sp.altitude_setpoint = altitude_setpoint_msl;
    lon_sp.pitch_sp = NAN;
    lon_sp.thrust_sp = NAN;

    lon_sp.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
    _fw_longitudinal_control_sp_pub->publish(lon_sp);
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
