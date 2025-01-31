/****************************************************************************
 * Copyright (c) 2023-2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/utils/subscription.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{
/** \ingroup vehicle_state
 *  @{
 */

/**
 * @brief Provides access to the vehicle's home position
 *
 * @ingroup vehicle_state
 */
class HomePosition : public Subscription<px4_msgs::msg::HomePosition>
{
public:
  explicit HomePosition(Context & context)
  : Subscription<px4_msgs::msg::HomePosition>(context,
      "fmu/out/home_position" + +px4_ros2::getMessageNameVersion<px4_msgs::msg::HomePosition>()) {}

  /**
   * @brief Get the vehicle's home position in local coordinates.
   *
   * @return the local coordinates of the home position (NED) [m]
   */
  Eigen::Vector3f localPosition() const
  {
    const px4_msgs::msg::HomePosition & home = last();
    return Eigen::Vector3f{home.x, home.y, home.z};
  }

  /**
   * @brief Get the vehicle's home position in global coordinates.
   *
   * @return the global coordinates of the home position (latitude [°], longitude [°], altitude [m AMSL])
   */
  Eigen::Vector3d globalPosition() const
  {
    const px4_msgs::msg::HomePosition & home = last();
    return Eigen::Vector3d{home.lat, home.lon, home.alt};
  }

  /**
   * @brief Get the vehicle's home position yaw.
   *
   * @return the yaw of the home position (NED) [rad]
   */
  float yaw() const
  {
    return last().yaw;
  }

  /**
   * @brief Check if vehicle's local home position is valid (xyz).
   *
   * @return true if the local position has been set, otherwise false
   */
  bool localPositionValid() const
  {
    return lastValid() && last().valid_lpos;
  }

  /**
   * @brief Check if vehicle's global horizontal home position is valid (lat, lon).
   *
   * @return true if latitude and longitude have been set, otherwise false
   */
  bool globaHorizontalPositionValid() const
  {
    return lastValid() && last().valid_hpos;
  }

  /**
   * @brief Check if vehicle's home position altitude is valid.
   *
   * @return true if altitude has been set, otherwise false
   */
  bool altitudeValid() const
  {
    return lastValid() && last().valid_alt;
  }

  /**
   * @brief Check if home position has been set manually.
   *
   * @return true if set manually, otherwise false
   */
  bool manualHome() const
  {
    return last().manual_home;
  }

};

/** @}*/
} /* namespace px4_ros2 */
