/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2_sdk/result.hpp>

namespace px4_ros2::sdk {

const char* resultToString(Result result) noexcept
{
  switch (result) {
    case Result::Success:
      return "Success";
    case Result::NotConnected:
      return "NotConnected";
    case Result::ConnectionError:
      return "ConnectionError";
    case Result::Busy:
      return "Busy";
    case Result::CommandDenied:
      return "CommandDenied";
    case Result::Timeout:
      return "Timeout";
    case Result::Unsupported:
      return "Unsupported";
    case Result::VersionMismatch:
      return "VersionMismatch";
    case Result::InvalidArgument:
      return "InvalidArgument";
    case Result::Unknown:
      return "Unknown";
  }
  return "Unknown";
}

Result fromCoreResult(px4_ros2::Result result) noexcept
{
  switch (result) {
    case px4_ros2::Result::Success:
      return Result::Success;
    case px4_ros2::Result::Rejected:
      return Result::CommandDenied;
    case px4_ros2::Result::Interrupted:
      return Result::ConnectionError;
    case px4_ros2::Result::Timeout:
      return Result::Timeout;
    case px4_ros2::Result::Deactivated:
      return Result::Busy;
    case px4_ros2::Result::ModeFailureOther:
      return Result::Unknown;
  }
  return Result::Unknown;
}

}  // namespace px4_ros2::sdk
