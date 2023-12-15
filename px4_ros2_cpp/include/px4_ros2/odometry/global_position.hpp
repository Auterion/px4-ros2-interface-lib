/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/odometry/subscription.hpp>

namespace px4_ros2
{
/** \ingroup odometry
 *  @{
 */

/**
 * Provides access to the vehicle's global position estimate
 */
class OdometryGlobalPosition : public Subscription<px4_msgs::msg::VehicleGlobalPosition>
{
public:
  explicit OdometryGlobalPosition(Context & context);

  /**
   * @brief Get the vehicle's global position.
   *
   * @returns a vector of (latitude [°], longitude [°], altitude [m AMSL])
   */
  Eigen::Vector3d position() const
  {
    const px4_msgs::msg::VehicleGlobalPosition & pos = last();
    return {pos.lat, pos.lon, pos.alt};
  }
};

/** @}*/
} /* namespace px4_ros2 */
