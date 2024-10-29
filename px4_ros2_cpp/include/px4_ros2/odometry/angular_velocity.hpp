/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/utils/subscription.hpp>

namespace px4_ros2
{
/** \ingroup odometry
 *  @{
 */

/**
 * @brief Provides access to the vehicle's angular velocity estimate
 */
class OdometryAngularVelocity : public Subscription<px4_msgs::msg::VehicleAngularVelocity>
{
public:
  explicit OdometryAngularVelocity(Context & context);

  /**
   * @brief Get the vehicle's angular velocity in FRD frame.
   *
   * @return the angular velocity
   */
  Eigen::Vector3f angularVelocityFrd() const
  {
    const px4_msgs::msg::VehicleAngularVelocity & av = last();
    return Eigen::Vector3f{av.xyz[0], av.xyz[1], av.xyz[2]};
  }
};

/** @}*/
} /* namespace px4_ros2 */
