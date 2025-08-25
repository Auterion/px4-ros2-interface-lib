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
  float desiredUpdateRateHz() override {return 30.f;}

  /**
   * @brief Send a rover throttle setpoint and a rover steering setpoint to the flight controller.
   *
   * @param throttle_body_x [-] Throttle setpoint along body X axis. Takes values in [-1 (Backwards), 1 (Forwards)].
   * @param normalized_steering_setpoint [-] Ackermann: Normalized steering angle, Differential/Mecanum: Normalized speed difference between the left and right wheels. Takes values in [-1 (Left), 1 (Right)].
   * @param throttle_body_y [-] Mecanum only: Throttle setpoint along body Y axis (Only relevant for mecanum rovers). Takes values in [-1 (Left), 1 (Right)].
  */
  void update(
    float throttle_body_x, float normalized_steering_setpoint,
    std::optional<float> throttle_body_y = {});

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::RoverThrottleSetpoint>::SharedPtr
    _rover_throttle_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::RoverSteeringSetpoint>::SharedPtr
    _rover_steering_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
