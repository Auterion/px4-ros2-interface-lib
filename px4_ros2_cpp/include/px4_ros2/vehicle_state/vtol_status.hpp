/****************************************************************************
 * Copyright (c) 2023-2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/vtol_vehicle_status.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/utils/subscription.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{
/** \ingroup vehicle_state
 *  @{
 */

/**
 * @brief Provides access to the vtol vehicle's status
 *
 * @ingroup vehicle_state
 */
class VtolStatus : public Subscription<px4_msgs::msg::VtolVehicleStatus>
{
public:
  explicit VtolStatus(Context & context)
  : Subscription<px4_msgs::msg::VtolVehicleStatus>(context,
      "fmu/out/vtol_vehicle_status" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VtolVehicleStatus>()) {}

  /**
   * @brief Check if vehicle is in an undefined state. This indicates the vehicle is not a VTOL.
   *
   * @return true if undefined state, false otherwise.
  */
  bool isUndefined() const
  {
    return last().vehicle_vtol_state ==
           px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_UNDEFINED;
  }

  /**
   * @brief Check if VTOL is transitioning to fixed-wing.
   *
   * @return true if transitioning to FW, false otherwise.
  */
  bool isTransitioningToFw() const
  {
    return last().vehicle_vtol_state ==
           px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_TRANSITION_TO_FW;
  }

  /**
   * @brief Check if VTOL is transitioning to multicopter.
   *
   * @return true if transitioning to MC, false otherwise.
  */
  bool isTransitioningToMc() const
  {
    return last().vehicle_vtol_state ==
           px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_TRANSITION_TO_MC;
  }

  /**
   * @brief Check if VTOL is in multicopter mode.
   *
   * @return true if in MC mode, false otherwise.
  */
  bool isMcMode() const
  {
    return last().vehicle_vtol_state == px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_MC;
  }

  /**
   * @brief Check if VTOL is in fixed-wing mode.
   *
   * @return true if in FW mode, false otherwise.
  */
  bool isFwMode() const
  {
    return last().vehicle_vtol_state == px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_FW;
  }

};

/** @}*/
} /* namespace px4_ros2 */
