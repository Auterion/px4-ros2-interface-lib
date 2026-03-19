/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_ros2/common/context.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/utils/vehicle_command_sender.hpp>

namespace px4_ros2 {
/** \ingroup vehicle_state
 *  @{
 */

/**
 * @brief Utility to set the vehicle's home position and GPS global origin.
 *
 * Sends VEHICLE_CMD_DO_SET_HOME and VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN
 * commands to PX4 via the vehicle_command topic and waits for an ACK.
 *
 * @ingroup vehicle_state
 */
class HomePositionSetter {
 public:
  explicit HomePositionSetter(Context& context);

  /**
   * @brief Set the home position to the current vehicle position.
   * @return Result::Success on ACK accepted, error otherwise.
   */
  Result setHomeToCurrentPosition();

  /**
   * @brief Set the home position to a specific global coordinate.
   *
   * @param latitude [deg]
   * @param longitude [deg]
   * @param altitude [m AMSL]
   * @return Result::Success on ACK accepted, error otherwise.
   */
  Result setHome(double latitude, double longitude, float altitude);

  /**
   * @brief Set the GPS global origin (EKF reference point).
   *
   * @param latitude [deg]
   * @param longitude [deg]
   * @param altitude [m AMSL]
   * @return Result::Success on ACK accepted, error otherwise.
   */
  Result setGpsGlobalOrigin(double latitude, double longitude, float altitude);

 private:
  Result sendCommand(uint32_t command, float param1 = 0.f, float param2 = 0.f, float param3 = 0.f,
                     float param4 = 0.f, double param5 = 0.0, double param6 = 0.0,
                     float param7 = 0.f);

  VehicleCommandSender _command_sender;
};

/** @}*/
}  // namespace px4_ros2
