/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/odometry/subscription.hpp>

namespace px4_ros2
{
/** \ingroup odometry
 *  @{
 */

/**
 * Provides access to the vehicle's local position estimate
 */
class OdometryLocalPosition : public Subscription<px4_msgs::msg::VehicleLocalPosition>
{
public:
  explicit OdometryLocalPosition(Context & context);

  bool positionXYValid() const
  {
    return last().xy_valid;
  }

  bool positionZValid() const
  {
    return last().z_valid;
  }

  Eigen::Vector3f position() const
  {
    const px4_msgs::msg::VehicleLocalPosition pos = last();
    return {pos.x, pos.y, pos.z};
  }

  bool velocityXYValid() const
  {
    return last().v_xy_valid;
  }

  bool velocityZValid() const
  {
    return last().v_z_valid;
  }
  Eigen::Vector3f velocity() const
  {
    const px4_msgs::msg::VehicleLocalPosition pos = last();
    return {pos.vx, pos.vy, pos.vz};
  }

  Eigen::Vector3f acceleration() const
  {
    const px4_msgs::msg::VehicleLocalPosition pos = last();
    return {pos.ax, pos.ay, pos.az};
  }
};

/** @}*/
} /* namespace px4_ros2 */
