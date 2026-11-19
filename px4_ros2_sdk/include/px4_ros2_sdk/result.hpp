/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_ros2/components/mode.hpp>

namespace px4_ros2::sdk {
/** \defgroup sdk High-level SDK facade
 *  @{
 */

/**
 * @brief Outcome of an SDK request.
 *
 * A Result reports the vehicle's acknowledgement / receipt of a request, not
 * that the physical action has completed. Long-running verbs report physical
 * completion separately (see the API expansion RFC).
 */
enum class Result {
  Success = 0,      ///< The command was accepted by the vehicle.
  NotConnected,     ///< No connection to the vehicle has been established.
  ConnectionError,  ///< The request was interrupted by a connection error.
  Busy,             ///< The vehicle is busy and cannot service the request.
  CommandDenied,    ///< The vehicle rejected the command.
  Timeout,          ///< No acknowledgement was received in time.
  Unsupported,      ///< The capability is not supported on this vehicle.
  VersionMismatch,  ///< The message set is incompatible with the vehicle.
  InvalidArgument,  ///< A caller argument was invalid.
  Unknown,          ///< An unmapped or unexpected error occurred.
};

/**
 * @brief Human-readable name for a Result value.
 */
const char* resultToString(Result result) noexcept;

/**
 * @brief Map a core px4_ros2::Result onto the SDK Result vocabulary.
 */
Result fromCoreResult(px4_ros2::Result result) noexcept;

/** @}*/
}  // namespace px4_ros2::sdk
