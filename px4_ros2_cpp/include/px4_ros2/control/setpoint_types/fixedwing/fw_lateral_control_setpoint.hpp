/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/fw_lateral_control_setpoint.hpp>
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
    class FwLateralControlSetpointType : public SetpointBase
    {
    public:
	explicit FwLateralControlSetpointType(Context & context);

	~FwLateralControlSetpointType() override = default;

	Configuration getConfiguration() override;

	/**
	 * @brief Update
	 *
	 * Unset optional values are not controlled
	 *
	 * @param position [m] NED earth-fixed frame
	 */
	void update(
			    const float course_setpoint, const float heading_setpoint, const float lateral_acceleration_setpoint
	);

	float desiredUpdateRateHz() override {return 30.f;}

    private:
	rclcpp::Node & _node;
	rclcpp::Publisher<px4_msgs::msg::FwLateralControlSetpoint >::SharedPtr
		_fw_lateral_control_sp_pub;
    };

/** @}*/
} /* namespace px4_ros2 */
