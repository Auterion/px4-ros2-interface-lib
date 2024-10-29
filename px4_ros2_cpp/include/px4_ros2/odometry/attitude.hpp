/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/subscription.hpp>

namespace px4_ros2
{
/** \ingroup odometry
 *  @{
 */

/**
 * @brief Provides access to the vehicle's attitude estimate
 */
class OdometryAttitude : public Subscription<px4_msgs::msg::VehicleAttitude>
{
public:
  explicit OdometryAttitude(Context & context);

  /**
   * @brief Get the vehicle's attitude.
   *
   * @return the attitude quaternion
   */
  Eigen::Quaternionf attitude() const
  {
    const px4_msgs::msg::VehicleAttitude & att = last();
    return Eigen::Quaternionf{att.q[0], att.q[1], att.q[2], att.q[3]};
  }

  /**
   * @brief Get the vehicle's roll in extrinsic RPY order.
   *
   * @return the attitude roll in radians within [-pi, pi]
   */
  float roll() const
  {
    return quaternionToRoll(attitude());
  }

  /**
   * @brief Get the vehicle's pitch in extrinsic RPY order.
   *
   * @return the attitude pitch in radians within [-pi, pi]
   */
  float pitch() const
  {
    return quaternionToPitch(attitude());
  }

  /**
   * @brief Get the vehicle's yaw in extrinsic RPY order.
   *
   * @return the attitude yaw in radians within [-pi, pi]
   */
  float yaw() const
  {
    return quaternionToYaw(attitude());
  }
};

/** @}*/
} /* namespace px4_ros2 */
