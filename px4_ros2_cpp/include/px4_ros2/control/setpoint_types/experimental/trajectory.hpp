/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <Eigen/Eigen>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types_experimental
 *  @{
 */

class TrajectorySetpointType : public SetpointBase
{
public:
  explicit TrajectorySetpointType(Context & context);

  ~TrajectorySetpointType() override = default;

  Configuration getConfiguration() override;

  void update(
    const Eigen::Vector3f & velocity_ned_m_s,
    const std::optional<Eigen::Vector3f> & acceleration_ned_m_s2 = {},
    std::optional<float> yaw_ned_rad = {},
    std::optional<float> yaw_rate_ned_rad_s = {});

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
