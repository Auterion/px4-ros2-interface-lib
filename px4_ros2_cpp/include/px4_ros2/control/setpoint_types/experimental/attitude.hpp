/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <Eigen/Eigen>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types_experimental
 *  @{
 */

class AttitudeSetpointType : public SetpointBase
{
public:
  explicit AttitudeSetpointType(Context & context);

  ~AttitudeSetpointType() override = default;

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override {return 200.F;}

  void update(
    const Eigen::Quaternionf & attidude_setpoint,
    const Eigen::Vector3f & thrust_setpoint_frd, float yaw_sp_move_rate_rad_s = 0.F);

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr
    _vehicle_attitude_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
