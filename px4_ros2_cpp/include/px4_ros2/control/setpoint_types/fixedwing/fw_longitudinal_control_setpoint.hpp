/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/fw_longitudinal_control_setpoint.hpp>
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
    class FwLongitudinalControlSetpointType : public SetpointBase
    {
    public:
	explicit FwLongitudinalControlSetpointType(Context & context);

	~FwLongitudinalControlSetpointType() override = default;

	Configuration getConfiguration() override;

	/**
	 * @brief Update
	 *
	 * Unset optional values are not controlled
	 *
	 */
	void update(
		const float altitude_setpoint_msl, const float height_rate_setpoint, const float equivalent_airspeed_setpoint
	);

	float desiredUpdateRateHz() override {return 30.f;}

    private:
	rclcpp::Node & _node;
	rclcpp::Publisher<px4_msgs::msg::FwLongitudinalControlSetpoint >::SharedPtr
		_fw_longitudinal_control_sp_pub;
    };

/** @}*/
} /* namespace px4_ros2 */
