/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/rover_attitude_setpoint.hpp>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types_rover
 *  @{
 */

/**
 * @brief Setpoint type for rover attitude control
*/
class RoverAttitudeSetpointType : public SetpointBase
{
public:
  explicit RoverAttitudeSetpointType(Context & context);

  ~RoverAttitudeSetpointType() override = default;

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override {return 100.f;}

  /**
   * @brief Send a rover attitude setpoint to the flight controller.
   *
   * @param yaw_setpoint [rad] [@range -inf, inf] [@frame NED] Yaw setpoint
  */
  void update(float yaw_setpoint);

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::RoverAttitudeSetpoint>::SharedPtr
    _rover_attitude_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
