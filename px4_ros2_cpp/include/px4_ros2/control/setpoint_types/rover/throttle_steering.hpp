/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/rover_throttle_setpoint.hpp>
#include <px4_msgs/msg/rover_steering_setpoint.hpp>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types_rover
 *  @{
 */

/**
 * @brief Setpoint type for rover throttle and steering control
*/
class RoverThrottleSteeringSetpointType : public SetpointBase
{
public:
  explicit RoverThrottleSteeringSetpointType(Context & context);

  ~RoverThrottleSteeringSetpointType() override = default;

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override {return 100.f;}

  /**
   * @brief Send a rover throttle setpoint and a rover steering setpoint to the flight controller.
   *
   * @param throttle_body_x [] [@range -1 (Backwards), 1 (Forwards)] [@frame Body] Throttle setpoint along body X axis
   * @param throttle_body_y [] [@range -1 (Left), 1 (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Throttle setpoint along body Y axis
   * @param normalized_steering_setpoint [@range -1 (Left), 1 (Right)] [@frame Body] Ackermann: Normalized steering angle, Differential/Mecanum: Normalized speed difference between the left and right wheels
  */
  void update(float throttle_body_x, float throttle_body_y, float normalized_steering_setpoint);

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::RoverThrottleSetpoint>::SharedPtr
    _rover_throttle_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::RoverSteeringSetpoint>::SharedPtr
    _rover_steering_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */