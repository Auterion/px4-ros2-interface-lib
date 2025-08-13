/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/rover_steering_setpoint.hpp>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types_rover
 *  @{
 */

/**
 * @brief Setpoint type for rover steering control
*/
class RoverSteeringSetpointType : public SetpointBase
{
public:
  explicit RoverSteeringSetpointType(Context & context);

  ~RoverSteeringSetpointType() override = default;

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override {return 100.f;}

  /**
   * @brief Send a rover steering setpoint to the flight controller.
   *
   * @param normalized_steering_setpoint [] [@range -1 (Left), 1 (Right)] [@frame Body] Ackermann: Normalized steering angle, Differential/Mecanum: Normalized speed difference between the left and right wheels
  */
  void update(float normalized_steering_setpoint);

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::RoverSteeringSetpoint>::SharedPtr
    _rover_steering_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
