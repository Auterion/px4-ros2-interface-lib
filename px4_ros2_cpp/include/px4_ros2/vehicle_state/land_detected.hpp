/****************************************************************************
 * Copyright (c) 2023-2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/utils/subscription.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{
/** \ingroup vehicle_state
 *  @{
 */

/**
 * @brief Provides access to the vehicle's status
 *
 * @ingroup vehicle_state
 */
class LandDetected : public Subscription<px4_msgs::msg::VehicleLandDetected>
{
public:
  explicit LandDetected(Context & context)
  : Subscription<px4_msgs::msg::VehicleLandDetected>(context,
      "fmu/out/vehicle_land_detected" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLandDetected>()) {}

  /**
   * @brief Check if vehicle is landed on the ground.
   *
   * @return true if landed, false otherwise
   */
  bool landed() const
  {
    return last().landed;
  }

};

/** @}*/
} /* namespace px4_ros2 */
