/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/odometry/subscription.hpp>

using namespace std::chrono_literals; // NOLINT

namespace px4_ros2
{
/** \ingroup odometry
 *  @{
 */

/**
 * Provides access to the vehicle's global position estimate
 */
class OdometryGlobalPosition
{
public:
  explicit OdometryGlobalPosition(Context & context);

  bool valid()
  {
    return _node.get_clock()->now() - _last_vehicle_global_position < 500ms;
  }

  Eigen::Vector3d position() const
  {
    return {_vehicle_global_position.lat, _vehicle_global_position.lon,
      _vehicle_global_position.alt};
  }

private:
  rclcpp::Node & _node;
  Subscription<px4_msgs::msg::VehicleGlobalPosition> _vehicle_global_position_sub;
  px4_msgs::msg::VehicleGlobalPosition _vehicle_global_position;
  rclcpp::Time _last_vehicle_global_position;
};

/** @}*/
} /* namespace px4_ros2 */
