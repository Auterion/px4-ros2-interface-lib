/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/fixed_wing_lateral_setpoint.hpp>
#include <px4_msgs/msg/fixed_wing_longitudinal_setpoint.hpp>
#include <px4_msgs/msg/lateral_control_limits.hpp>
#include <px4_msgs/msg/longitudinal_control_limits.hpp>
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
 * @brief Setpoint type for fixedwing control
*/
    class FwLatLonSetpointType : public SetpointBase
    {
    public:
	explicit FwLatLonSetpointType(Context & context);

	~FwLatLonSetpointType() override = default;

	Configuration getConfiguration() override;

	/**
	 * @brief Update
	 *
	 * Unset optional values are not controlled
	 *
	 */
	void update(
			const float course_setpoint,
			const float airspeed_direction, 
			const float lateral_acceleration_setpoint, 
			const float altitude_setpoint_msl,
			const float height_rate_setpoint,
			const float equivalent_airspeed_setpoint,
			const std::optional<float> & min_pitch = {},
			const std::optional<float> & max_pitch = {},
			const std::optional<float> & min_throttle = {},
			const std::optional<float> & max_throttle = {},
			const std::optional<float> & min_equivalent_airspeed = {},
			const std::optional<float> & max_equivalent_airspeed = {},
			const std::optional<float> & max_lat_acc = {},
			const std::optional<float> & target_climb_rate = {},
			const std::optional<float> & target_sink_rate = {}
	);

	float desiredUpdateRateHz() override {return 30.f;}

    private:
	rclcpp::Node & _node;
	rclcpp::Publisher<px4_msgs::msg::FixedWingLateralSetpoint >::SharedPtr
		_fw_lateral_sp_pub;
	rclcpp::Publisher<px4_msgs::msg::FixedWingLongitudinalSetpoint >::SharedPtr
		_fw_longitudinal_sp_pub;
	
	rclcpp::Publisher<px4_msgs::msg::LateralControlLimits >::SharedPtr
		_lateral_control_limits_pub;
	rclcpp::Publisher<px4_msgs::msg::LongitudinalControlLimits >::SharedPtr
		_longitudinal_control_limits_pub;
    };

/** @}*/
} /* namespace px4_ros2 */