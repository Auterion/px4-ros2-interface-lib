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

/**
 * @brief Setpoint type for direct attitude control
*/
class AttitudeSetpointType : public SetpointBase
{
public:
  explicit AttitudeSetpointType(Context & context);

  ~AttitudeSetpointType() override = default;

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override {return 100.f;}

  void update(
    const Eigen::Quaternionf & attitude_setpoint,
    const Eigen::Vector3f & thrust_setpoint_frd, float yaw_sp_move_rate_rad_s = 0.f);

  /**
   * @brief Send an attitude setpoint to the flight controller.
   * Euler angles follow the RPY extrinsic Tait-Bryan convention (equiv. YPR intrinsic)
   *
   * @param roll Attitude setpoint roll angle [rad]
   * @param pitch Attitude setpoint pitch angle [rad]
   * @param yaw Attitude setpoint yaw angle [rad]
   * @param thrust_setpoint_body Throttle demand [-1, 1]^3
   * @param yaw_sp_move_rate_rad_s Yaw setpoint move rate [rad/s]
  */
  void update(
    float roll, float pitch, float yaw,
    const Eigen::Vector3f & thrust_setpoint_body, float yaw_sp_move_rate_rad_s = 0.f);

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr
    _vehicle_attitude_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
