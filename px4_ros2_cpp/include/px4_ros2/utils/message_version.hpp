/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

/**
 * @defgroup message_version Message Version
 * @ingroup utils
 * This group contains helper functions to handle message versioning.
 */

#pragma once

#include <string>
#include <type_traits>

namespace px4_ros2
{

/**
 * @brief Trait to check if a message type `T` has a `MESSAGE_VERSION` constant.
 *
 * If `T` has `MESSAGE_VERSION`, `HasMessageVersion<T>::value` is `true`;
 * otherwise it is `false`.
 *
 * @tparam T The message type to check.
 */
template<typename T, typename = void>
struct HasMessageVersion : std::false_type {};

/// Specialization for types that have a `MESSAGE_VERSION` constant.
template<typename T>
struct HasMessageVersion<T, std::void_t<decltype(T::MESSAGE_VERSION)>>: std::true_type {};

/**
 * @brief Retrieves the version suffix for a given message type.
 *
 * @tparam T The message type, which may or may not define a `MESSAGE_VERSION` constant.
 * @return std::string The version suffix (e.g., "_v1") or an empty string if `MESSAGE_VERSION` is `0` or undefined.
 */
template<typename T>
std::string getMessageNameVersion()
{
  if constexpr (HasMessageVersion<T>::value) {
    if (T::MESSAGE_VERSION == 0) {return "";}
    return "_v" + std::to_string(T::MESSAGE_VERSION);
  } else {
    return "";
  }
}


} // namespace px4_ros2
