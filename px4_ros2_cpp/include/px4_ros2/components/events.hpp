/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/arming_check_reply.hpp>

namespace px4_ros2::events
{

using EventType = px4_msgs::msg::Event;

enum class LogLevel : uint8_t
{
  Emergency = 0,
  Alert = 1,
  Critical = 2,
  Error = 3,
  Warning = 4,
  Notice = 5,
  Info = 6,
  Debug = 7,
  Protocol = 8,
  Disabled = 9,

  Count
};

enum class LogLevelInternal : uint8_t
{
  Emergency = 0,
  Alert = 1,
  Critical = 2,
  Error = 3,
  Warning = 4,
  Notice = 5,
  Info = 6,
  Debug = 7,
  Protocol = 8,
  Disabled = 9,

  Count
};

using Log = LogLevel;
using LogInternal = LogLevelInternal;

struct LogLevels
{
  LogLevels() = default;
  LogLevels(Log external_level) // NOLINT
  : external(external_level), internal(static_cast<LogInternal>(external_level)) {}
  LogLevels(Log external_level, LogInternal internal_level)
  : external(external_level), internal(internal_level) {}

  Log external{Log::Info};
  LogInternal internal{LogInternal::Info};
};

namespace util
{

// source: https://gist.github.com/ruby0x1/81308642d0325fd386237cfa3b44785c
constexpr uint32_t kVal32Const = 0x811c9dc5;
constexpr uint32_t kPrime32Const = 0x1000193;
inline constexpr uint32_t hash32Fnv1aConst(
  const char * const str,
  const uint32_t value = kVal32Const) noexcept
{
  return (str[0] == '\0') ? value : hash32Fnv1aConst(
    &str[1], (value ^ static_cast<uint32_t>(
      str[0])) * kPrime32Const);
}

template<typename T>
inline constexpr void fillEventArguments(uint8_t * buf, T arg)
{
  // This assumes we're on little-endian
  memcpy(buf, &arg, sizeof(T));
}

template<typename T, typename ... Args>
inline constexpr void fillEventArguments(uint8_t * buf, T arg, Args... args)
{
  fillEventArguments(buf, arg);
  fillEventArguments(buf + sizeof(T), args ...);
}

constexpr unsigned sizeofArguments() {return 0;}

template<typename T, typename ... Args>
constexpr unsigned sizeofArguments(const T & t, const Args &... args)
{
  return sizeof(T) + sizeofArguments(args ...);
}

} // namespace util

/**
 * Generate event ID from an event name
 */
template<size_t N>
constexpr uint32_t ID(const char (& name)[N]) // NOLINT(readability-identifier-naming)
{
  // Note: the generated ID must match with the python generator under Tools/px4events
  const uint32_t component_id = 1U << 24;       // autopilot component
  return (0xffffff & util::hash32Fnv1aConst(name)) | component_id;
}


} // namespace px4_ros2::events
