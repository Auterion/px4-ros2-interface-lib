/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/utils/subscription.hpp>

namespace px4_ros2
{
/** \ingroup odometry
 *  @{
 */

/**
 * @brief Provides access to the vehicle's local position estimate
 */
class OdometryLocalPosition : public Subscription<px4_msgs::msg::VehicleLocalPosition>
{
public:
  /**
   * setting local_position_is_optional to true allows to create a mode that uses local position
   * data but doesn't necessarly require local position to be available (for instance if the mode reads
   * altitude data from vehicle_local_position). Reading XY position without a correct source of
   * positional data is not recommended
   */
  explicit OdometryLocalPosition(Context & context, bool local_position_is_optional = false);

  bool positionXYValid() const
  {
    return lastValid() && last().xy_valid;
  }

  bool positionZValid() const
  {
    return lastValid() && last().z_valid;
  }

  Eigen::Vector3f positionNed() const
  {
    const px4_msgs::msg::VehicleLocalPosition & pos = last();
    return {pos.x, pos.y, pos.z};
  }

  bool velocityXYValid() const
  {
    return lastValid() && last().v_xy_valid;
  }

  bool velocityZValid() const
  {
    return lastValid() && last().v_z_valid;
  }
  Eigen::Vector3f velocityNed() const
  {
    const px4_msgs::msg::VehicleLocalPosition & pos = last();
    return {pos.vx, pos.vy, pos.vz};
  }

  Eigen::Vector3f accelerationNed() const
  {
    const px4_msgs::msg::VehicleLocalPosition & pos = last();
    return {pos.ax, pos.ay, pos.az};
  }

  /**
   * @brief Get the vehicle's heading relative to NED earth-fixed frame.
   *
   * @return The vehicle's yaw in radians within [-pi, pi] using extrinsic RPY order
  */
  float heading() const
  {
    const px4_msgs::msg::VehicleLocalPosition & pos = last();
    return pos.heading;
  }

  float distanceGround() const
  {
    const px4_msgs::msg::VehicleLocalPosition & pos = last();
    return pos.dist_bottom;
  }
};

/** @}*/
} /* namespace px4_ros2 */
