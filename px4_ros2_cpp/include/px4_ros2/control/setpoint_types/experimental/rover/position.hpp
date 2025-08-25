/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/rover_position_setpoint.hpp>
#include <Eigen/Eigen>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types_rover
 *  @{
 */

/**
 * @brief Setpoint type for rover position control
*/
class RoverPositionSetpointType : public SetpointBase
{
public:
  explicit RoverPositionSetpointType(Context & context);

  ~RoverPositionSetpointType() override = default;

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override {return 30.f;}

  /**
   * @brief Send a rover position setpoint to the flight controller.
   *
   * @param position_ned [m] Target position in NED frame. Takes values in [-inf, inf].
   * @param start_ned [m] Start position which specifies a line for the rover to track (Optional, defaults to vehicle position). Takes values in [-inf, inf].
   * @param cruising_speed [m/s] Cruising speed (Optional, defaults to maximum speed). Takes values in [0, inf].
   * @param arrival_speed [m/s] Speed the rover should arrive at the target with (Optional, defaults to zero). Takes values in [0, inf].
   * @param yaw [rad] Mecanum only: Specify vehicle yaw during travel (Optional, defaults to vehicle yaw). Takes values in [-pi,pi].
  */
  void update(
    const Eigen::Vector2f & position_ned, const std::optional<Eigen::Vector2f> & start_ned = {},
    std::optional<float> cruising_speed = {}, std::optional<float> arrival_speed = {},
    std::optional<float> yaw = {});

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::RoverPositionSetpoint>::SharedPtr
    _rover_position_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
