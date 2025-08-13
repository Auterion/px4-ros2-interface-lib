/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/rover_velocity_setpoint.hpp>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types_rover
 *  @{
 */

/**
 * @brief Setpoint type for rover velocity control
*/
class RoverVelocitySetpointType : public SetpointBase
{
public:
  explicit RoverVelocitySetpointType(Context & context);

  ~RoverVelocitySetpointType() override = default;

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override {return 100.f;}

  /**
   * @brief Send a rover velocity setpoint to the flight controller.
   *
   * @param speed [m/s] [@range -inf (Backwards), inf (Forwards)] Speed setpoint
   * @param bearing [rad] [@range -pi,pi] [@frame NED] [@invalid: NaN, speed is defined in body x direction] Bearing setpoint
   * @param yaw [rad] [@range -pi, pi] [@frame NED] [@invalid NaN, Defaults to vehicle yaw] Mecanum only: Yaw setpoint
  */
  void update(float speed, float bearing, float yaw);

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::RoverVelocitySetpoint>::SharedPtr
    _rover_velocity_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
