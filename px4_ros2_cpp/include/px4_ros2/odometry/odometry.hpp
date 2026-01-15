/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/subscription.hpp>

namespace px4_ros2 {
/** \ingroup odometry
 *  @{
 */

/**
 * @brief Provides access to the vehicle's odometry estimate
 * @details Use this wrapper when you need position, velocity, attitude, and angular rates
 * to be strictly time-aligned from the same odometry message.
 * This class consolidates the odometry snapshot so advanced control (e.g. MPC) or end-to-end
 * reinforcement learning pipelines can consume a consistent state vector at a single timestamp.
 */
class Odometry : public Subscription<px4_msgs::msg::VehicleOdometry> {
 public:
  explicit Odometry(Context& context);

  /**
   * @brief Get the vehicle's position in the North-East-Down (NED) local frame.
   * * @return The [North, East, Down] position in meters.
   */
  Eigen::Vector3f positionNed() const
  {
    const px4_msgs::msg::VehicleOdometry& odom = last();
    return {odom.position[0], odom.position[1], odom.position[2]};
  }

  /**
   * @brief Get the vehicle's linear velocity in the North-East-Down (NED) frame.
   * * @note By default, PX4 uses VELOCITY_FRAME_NED for odometry updates.
   * @return the velocity.
   */
  Eigen::Vector3f velocityNed() const
  {
    const px4_msgs::msg::VehicleOdometry& odom = last();
    return {odom.velocity[0], odom.velocity[1], odom.velocity[2]};
  }

  /**
   * @brief Get the vehicle's angular velocity in FRD frame.
   *
   * @return the angular velocity
   */
  Eigen::Vector3f angularVelocityFrd() const
  {
    const px4_msgs::msg::VehicleOdometry& odom = last();
    return {odom.angular_velocity[0], odom.angular_velocity[1], odom.angular_velocity[2]};
  }

  /**
   * @brief Get the vehicle's attitude.
   *
   * @return the attitude quaternion
   */
  Eigen::Quaternionf attitude() const
  {
    const px4_msgs::msg::VehicleOdometry& odom = last();
    return Eigen::Quaternionf{odom.q[0], odom.q[1], odom.q[2], odom.q[3]};
  }

  /**
   * @brief Get the vehicle's roll in extrinsic RPY order.
   *
   * @return the attitude roll in radians within [-pi, pi]
   */
  float roll() const { return quaternionToRoll(attitude()); }

  /**
   * @brief Get the vehicle's pitch in extrinsic RPY order.
   *
   * @return the attitude pitch in radians within [-pi, pi]
   */
  float pitch() const { return quaternionToPitch(attitude()); }

  /**
   * @brief Get the vehicle's yaw in extrinsic RPY order.
   *
   * @return the attitude yaw in radians within [-pi, pi]
   */
  float yaw() const { return quaternionToYaw(attitude()); }
};

/** @}*/
} /* namespace px4_ros2 */
