/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/rover_throttle_setpoint.hpp>
#include <px4_msgs/msg/rover_attitude_setpoint.hpp>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types_rover
 *  @{
 */

/**
 * @brief Setpoint type for rover throttle and attitude control
*/
class RoverThrottleAttitudeSetpointType : public SetpointBase
{
public:
  explicit RoverThrottleAttitudeSetpointType(Context & context);

  ~RoverThrottleAttitudeSetpointType() override = default;

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override {return 100.f;}

  /**
   * @brief Send a rover throttle setpoint and a rover attitude setpoint to the flight controller.
   *
   * @param throttle_body_x [] [@range -1 (Backwards), 1 (Forwards)] [@frame Body] Throttle setpoint along body X axis
   * @param throttle_body_y [] [@range -1 (Left), 1 (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Throttle setpoint along body Y axis
   * @param yaw_setpoint [rad] [@range -inf, inf] [@frame NED] Yaw setpoint
  */
  void update(float throttle_body_x, float throttle_body_y, float yaw_setpoint);

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::RoverThrottleSetpoint>::SharedPtr
    _rover_throttle_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::RoverAttitudeSetpoint>::SharedPtr
    _rover_attitude_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */

