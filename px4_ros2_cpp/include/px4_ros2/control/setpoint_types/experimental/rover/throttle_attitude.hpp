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
  float desiredUpdateRateHz() override {return 30.f;}

  /**
   * @brief Send a rover throttle setpoint and a rover attitude setpoint to the flight controller.
   *
   * @param throttle_body_x [-] Throttle setpoint along body X axis. Takes values in [-1 (Backwards), 1 (Forwards)].
   * @param yaw_setpoint [rad] Yaw setpoint in NED frame. Takes values in [-pi, pi].
   * @param throttle_body_y [-] Mecanum only: Throttle setpoint along body Y axis (Only relevant for mecanum rovers). Takes values in [-1 (Left), 1 (Right)].
  */
  void update(float throttle_body_x, float yaw_setpoint, std::optional<float> throttle_body_y = {});

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::RoverThrottleSetpoint>::SharedPtr
    _rover_throttle_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::RoverAttitudeSetpoint>::SharedPtr
    _rover_attitude_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
