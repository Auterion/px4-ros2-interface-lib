/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/utils/subscription.hpp>

namespace px4_ros2 {
/** \ingroup odometry
 *  @{
 */

/**
 * @brief Provides access to the vehicle's angular velocity estimate
 *
 * @note PX4 does not publish `/fmu/out/vehicle_angular_velocity` to ROS 2 by default, to save
 * bandwidth. Enable it in PX4's `src/modules/uxrce_dds_client/dds_topics.yaml`, ideally with a
 * `rate_limit`, since PX4 publishes it at up to `IMU_GYRO_RATEMAX`. Without it, no message is
 * received and angularVelocityFrd() throws.
 */
class OdometryAngularVelocity : public Subscription<px4_msgs::msg::VehicleAngularVelocity> {
 public:
  explicit OdometryAngularVelocity(Context& context);

  /**
   * @brief Get the vehicle's angular velocity in FRD frame.
   *
   * @return the angular velocity
   */
  Eigen::Vector3f angularVelocityFrd() const
  {
    const px4_msgs::msg::VehicleAngularVelocity& av = last();
    return Eigen::Vector3f{av.xyz[0], av.xyz[1], av.xyz[2]};
  }
};

/** @}*/
} /* namespace px4_ros2 */
