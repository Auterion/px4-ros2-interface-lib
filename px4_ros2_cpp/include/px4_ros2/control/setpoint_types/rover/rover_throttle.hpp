/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/rover_throttle_setpoint.hpp>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types_rover
 *  @{
 */

/**
 * @brief Setpoint type for rover throttle control
*/
class RoverThrottleSetpointType : public SetpointBase
{
public:
  explicit RoverThrottleSetpointType(Context & context);

  ~RoverThrottleSetpointType() override = default;

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override {return 100.f;}

  /**
   * @brief Send a rover throttle setpoint to the flight controller.
   *
   * @param throttle_body_x [] [@range -1 (Backwards), 1 (Forwards)] [@frame Body] Throttle setpoint along body X axis
   * @param throttle_body_y [] [@range -1 (Left), 1 (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Throttle setpoint along body Y axis
  */
  void update(float throttle_body_x, float throttle_body_y);

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::RoverThrottleSetpoint>::SharedPtr
    _rover_throttle_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
