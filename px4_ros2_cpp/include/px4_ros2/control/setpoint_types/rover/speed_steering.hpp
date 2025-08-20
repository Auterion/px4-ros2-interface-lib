/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/rover_speed_setpoint.hpp>
#include <px4_msgs/msg/rover_steering_setpoint.hpp>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types_rover
 *  @{
 */

/**
 * @brief Setpoint type for rover speed and steering control
*/
class RoverSpeedSteeringSetpointType : public SetpointBase
{
public:
  explicit RoverSpeedSteeringSetpointType(Context & context);

  ~RoverSpeedSteeringSetpointType() override = default;

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override {return 100.f;}

  /**
   * @brief Send a rover speed setpoint and a rover steering setpoint to the flight controller.
   *
   * @param speed_body_x [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Speed setpoint in body x direction
   * @param speed_body_y [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Speed setpoint in body y direction
   * @param normalized_steering_setpoint [@range -1 (Left), 1 (Right)] [@frame Body] Ackermann: Normalized steering angle, Differential/Mecanum: Normalized speed difference between the left and right wheels
  */
  void update(float speed_body_x, float speed_body_y, float normalized_steering_setpoint);

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::RoverSpeedSetpoint>::SharedPtr
    _rover_speed_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::RoverSteeringSetpoint>::SharedPtr
    _rover_steering_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
