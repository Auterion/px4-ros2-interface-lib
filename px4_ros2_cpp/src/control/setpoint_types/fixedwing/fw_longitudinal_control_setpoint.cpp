/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/fixedwing/fw_longitudinal_control_setpoint.hpp>
#include <px4_msgs/msg/fw_longitudinal_control_setpoint.hpp>


namespace px4_ros2
{

    FwLongitudinalControlSetpointType::FwLongitudinalControlSetpointType(Context & context)
	    : SetpointBase(context), _node(context.node())
    {
	_fw_longitudinal_control_sp_pub =
		context.node().create_publisher<px4_msgs::msg::FwLongitudinalControlSetpoint>(
			context.topicNamespacePrefix() + "fmu/in/fw_longitudinal_control_setpoint", 1);
    }

    void FwLongitudinalControlSetpointType::update(
	const float altitude_setpoint_msl, const float height_rate_setpoint, const float equivalent_airspeed_setpoint)
    {
	onUpdate();

	px4_msgs::msg::FwLongitudinalControlSetpoint sp{};

	sp.altitude_setpoint = altitude_setpoint_msl;
	sp.height_rate_setpoint = height_rate_setpoint;
	sp.equivalent_airspeed_setpoint = equivalent_airspeed_setpoint;

	sp.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
	_fw_longitudinal_control_sp_pub->publish(sp);
    }

    SetpointBase::Configuration FwLongitudinalControlSetpointType::getConfiguration()
    {
	Configuration config{};
	config.control_allocation_enabled = true;
	config.rates_enabled = true;
	config.attitude_enabled = true;
	config.altitude_enabled = true;
	config.acceleration_enabled = false;
	config.velocity_enabled = false;
	config.position_enabled = false;
	config.climb_rate_enabled = true;
	return config;
    }

} // namespace px4_ros2
