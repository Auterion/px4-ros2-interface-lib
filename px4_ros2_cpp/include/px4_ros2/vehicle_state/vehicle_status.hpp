/****************************************************************************
 * Copyright (c) 2023-2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/vehicle_status.hpp>
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
class VehicleStatus : public Subscription<px4_msgs::msg::VehicleStatus>
{
public:
  explicit VehicleStatus(Context & context)
  : Subscription<px4_msgs::msg::VehicleStatus>(context,
      "fmu/out/vehicle_status" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>()) {}

  /**
   * @brief Get the vehicle's arming status.
   *
   * @return true if the vehicle is armed, false otherwise
  */
  bool armed() const
  {
    return last().arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  }

  /**
   * @brief Get the vehicle's current active flight mode.
   *
   * @return the int value describing the mode
   * @see navigation state values: https://github.com/PX4/PX4-Autopilot/blob/v1.15.0/msg/VehicleStatus.msg#L34-L65
  */
  uint8_t navState() const
  {
    return last().nav_state;
  }

};

/** @}*/
} /* namespace px4_ros2 */
