/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/goto_setpoint.hpp>
#include <Eigen/Eigen>
#include <optional>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types
 *  @{
 */

class GotoSetpointType : public SetpointBase
{
public:
  explicit GotoSetpointType(Context & context);

  ~GotoSetpointType() override = default;

  Configuration getConfiguration() override;

  /**
   * @brief Go-to setpoint update
   *
   * Unset optional values are not controlled
   *
   * @param position [m] NED earth-fixed frame
   * @param heading [rad] from North
   * @param max_horizontal_speed [m/s] in NE-datum of NED earth-fixed frame
   * @param max_vertical_speed [m/s] in D-axis of NED earth-fixed frame
   * @param max_heading_rate [rad/s] about D-axis of NED earth-fixed frame
   */
  void update(
    const Eigen::Vector3f & position,
    const std::optional<float> & heading = {},
    const std::optional<float> & max_horizontal_speed = {},
    const std::optional<float> & max_vertical_speed = {},
    const std::optional<float> & max_heading_rate = {}
  );

  float desiredUpdateRateHz() override {return 30.F;}

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::GotoSetpoint>::SharedPtr
    _goto_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
