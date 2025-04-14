/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/utils/subscription.hpp>

namespace px4_ros2
{
/** \ingroup odometry
 *  @{
 */

/**
 * @brief Provides access to the vehicle's sensor data 
 */
class SensorData : public Subscription<px4_msgs::msg::SensorCombined>
{
public:
  explicit SensorData(Context & context);

  /**
   * @brief Get the vehicle's acceleration data combined from the different IMUs in FRD frame.
   *
   */
  Eigen::Vector3f accelerationFRD() const
  {
    const px4_msgs::msg::SensorCombined & sens = last();
    return {sens.accelerometer_m_s2[0], sens.accelerometer_m_s2[1], sens.accelerometer_m_s2[2]};
  }

  Eigen::Vector3f gyroFRD() const
  {
    const px4_msgs::msg::SensorCombined & sens = last();
    return {sens.gyro_rad[0], sens.gyro_rad[1], sens.gyro_rad[2]};
  }
};

/** @}*/
} /* namespace px4_ros2 */
