/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/rover_rate_setpoint.hpp>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types_rover
 *  @{
 */

/**
 * @brief Setpoint type for rover rate control
*/
class RoverRateSetpointType : public SetpointBase
{
public:
  explicit RoverRateSetpointType(Context & context);

  ~RoverRateSetpointType() override = default;

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override {return 100.f;}

  /**
   * @brief Send a rover rate setpoint to the flight controller.
   *
   * @param yaw_rate_setpoint [rad/s] [@range -inf, inf] [@frame NED] Yaw rate setpoint
  */
  void update(float yaw_rate_setpoint);

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::RoverRateSetpoint>::SharedPtr
    _rover_rate_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
