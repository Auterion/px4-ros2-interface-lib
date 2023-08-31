/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <px4_sdk/common/context.hpp>

namespace px4_sdk
{

class OdometryLocalPosition
{
public:
  explicit OdometryLocalPosition(Context & context);

  bool positionXyValid() const
  {
    return _vehicle_local_position.xy_valid;
  }

  bool positionZValid() const
  {
    return _vehicle_local_position.z_valid;
  }

  Eigen::Vector3f position() const
  {
    return {_vehicle_local_position.x, _vehicle_local_position.y, _vehicle_local_position.z};
  }

  bool velocityXYValid() const
  {
    return _vehicle_local_position.v_xy_valid;
  }

  bool velocityZValid() const
  {
    return _vehicle_local_position.v_z_valid;
  }
  Eigen::Vector3f velocity() const
  {
    return {_vehicle_local_position.vx, _vehicle_local_position.vy, _vehicle_local_position.vz};
  }

  Eigen::Vector3f acceleration() const
  {
    return {_vehicle_local_position.ax, _vehicle_local_position.ay, _vehicle_local_position.az};
  }

private:
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _vehicle_local_position_sub;
  px4_msgs::msg::VehicleLocalPosition _vehicle_local_position;
};

} /* namespace px4_sdk */
