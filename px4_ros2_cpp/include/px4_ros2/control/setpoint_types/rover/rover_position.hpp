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
  float desiredUpdateRateHz() override {return 100.f;}

  /**
   * @brief Send a rover position setpoint to the flight controller.
   *
   * @param position_ned [m] [@range -inf, inf] [@frame NED] Target position
   * @param start_ned [m] [@range -inf, inf] [@frame NED] [@invalid NaN Defaults to vehicle position] Start position which specifies a line for the rover to track
   * @param cruising_speed [m/s] [@range 0, inf] [@invalid NaN Defaults to maximum speed] Cruising speed
   * @param arrival_speed [m/s] [@range 0, inf] [@invalid NaN Defaults to 0] Speed the rover should arrive at the target with
   * @param yaw [rad] [@range -pi,pi] [@frame NED] [@invalid NaN Defaults to vehicle yaw] Mecanum only: Specify vehicle yaw during travel
  */
  void update(
    const Eigen::Vector2f & position_ned, const Eigen::Vector2f & start_ned,
    float cruising_speed, float arrival_speed, float yaw);

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::RoverPositionSetpoint>::SharedPtr
    _rover_position_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
