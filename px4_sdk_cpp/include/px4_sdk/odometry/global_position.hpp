/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <px4_sdk/common/context.hpp>

using namespace std::chrono_literals; // NOLINT

namespace px4_sdk
{

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
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr _vehicle_global_position_sub;
  px4_msgs::msg::VehicleGlobalPosition _vehicle_global_position;
  rclcpp::Time _last_vehicle_global_position;
};

} /* namespace px4_sdk */
