/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/fw_global_goto_setpoint.hpp>
#include <Eigen/Eigen>
#include <optional>

#include <px4_ros2/common/setpoint_base.hpp>
#include <px4_ros2/utils/geodesic.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types
 *  @{
 */

/**
 * @brief Setpoint type for smooth position and heading control
*/
    class FwGotoGlobalSetpointType : public SetpointBase
    {
    public:
	explicit FwGotoGlobalSetpointType(Context & context);

	~FwGotoGlobalSetpointType() override = default;

	Configuration getConfiguration() override;

	/**
	 * @brief Go-to setpoint update
	 *
	 * Unset optional values are not controlled
	 *
	 * @param position [m] NED earth-fixed frame
	 */
	void update(
		const Eigen::Vector3d & position
	);

	float desiredUpdateRateHz() override {return 30.f;}

    private:
	rclcpp::Node & _node;
	rclcpp::Publisher<px4_msgs::msg::FwGlobalGotoSetpoint>::SharedPtr
		_goto_setpoint_pub;
    };

/** @}*/
} /* namespace px4_ros2 */
