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
  float desiredUpdateRateHz() override {return 30.f;}

  /**
   * @brief Send a rover speed setpoint and a rover steering setpoint to the flight controller.
   *
   * @param speed_body_x [m/s] Speed setpoint in body x direction. Takes values in [-inf (Backwards), inf (Forwards)].
   * @param normalized_steering_setpoint [-] Ackermann: Normalized steering angle, Differential/Mecanum: Normalized speed difference between the left and right wheels. Takes values in [-1 (Left), 1 (Right)].
   * @param speed_body_y [m/s] Mecanum only: Speed setpoint in body y direction (Only relevant for mecanum rovers). Takes values in [-inf (Left), inf (Right)].
  */
  void update(
    float speed_body_x, float normalized_steering_setpoint,
    std::optional<float> speed_body_y = {});

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::RoverSpeedSetpoint>::SharedPtr
    _rover_speed_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::RoverSteeringSetpoint>::SharedPtr
    _rover_steering_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
