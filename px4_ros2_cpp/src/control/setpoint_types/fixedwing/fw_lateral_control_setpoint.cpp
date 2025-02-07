/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/fixedwing/fw_lateral_control_setpoint.hpp>
#include <px4_msgs/msg/fw_lateral_control_setpoint.hpp>


namespace px4_ros2
{

    FwLateralControlSetpointType::FwLateralControlSetpointType(Context & context)
	    : SetpointBase(context), _node(context.node())
    {
	_fw_lateral_control_sp_pub=
		context.node().create_publisher<px4_msgs::msg::FwLateralControlSetpoint>(
			context.topicNamespacePrefix() + "fmu/in/fw_lateral_control_setpoint", 1);
    }

    void FwLateralControlSetpointType::update(
	    const float course_setpoint, const float heading_setpoint, const float lateral_acceleration_setpoint)
    {
	onUpdate();

	px4_msgs::msg::FwLateralControlSetpoint sp{};
	sp.course_setpoint = course_setpoint;
	sp.airspeed_reference_direction = heading_setpoint;
	sp.lateral_acceleration_setpoint = lateral_acceleration_setpoint;

	sp.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
	_fw_lateral_control_sp_pub->publish(sp);
    }

    SetpointBase::Configuration FwLateralControlSetpointType::getConfiguration()
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
