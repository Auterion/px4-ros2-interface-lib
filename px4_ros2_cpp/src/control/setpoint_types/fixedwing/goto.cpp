//
// Created by roman on 12.11.24.
//
/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/fixedwing/goto.hpp>
#include <px4_msgs/msg/fw_global_goto_setpoint.hpp>


namespace px4_ros2
{

    FwGotoGlobalSetpointType::FwGotoGlobalSetpointType(Context & context)
	    : SetpointBase(context), _node(context.node())
    {
	_goto_setpoint_pub =
		context.node().create_publisher<px4_msgs::msg::FwGlobalGotoSetpoint>(
			context.topicNamespacePrefix() + "fmu/in/fw_global_goto_setpoint", 1);
    }

    void FwGotoGlobalSetpointType::update(
	    const Eigen::Vector3d & position)
    {
	onUpdate();

	px4_msgs::msg::FwGlobalGotoSetpoint sp{};

	// setpoints
	sp.lat = position(0);
	sp.lon = position(1);
	sp.alt_msl = position(2);

	sp.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
	_goto_setpoint_pub->publish(sp);
    }

    SetpointBase::Configuration FwGotoGlobalSetpointType::getConfiguration()
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
