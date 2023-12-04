/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/vehicle_odometry.hpp>

namespace px4_ros2
{

enum class PoseFrame
{
  Unknown,
  LocalNED,
  LocalFRD
};

enum class VelocityFrame
{
  Unknown,
  LocalNED,
  LocalFRD,
  BodyFRD
};

constexpr inline uint8_t poseFrameToMessageFrame(px4_ros2::PoseFrame frame) noexcept
{
  switch (frame) {
    case PoseFrame::Unknown: return px4_msgs::msg::VehicleOdometry::POSE_FRAME_UNKNOWN;

    case PoseFrame::LocalNED: return px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    case PoseFrame::LocalFRD: return px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
  }

  return px4_msgs::msg::VehicleOdometry::POSE_FRAME_UNKNOWN;
}

constexpr inline uint8_t velocityFrameToMessageFrame(px4_ros2::VelocityFrame frame) noexcept
{
  switch (frame) {
    case VelocityFrame::Unknown: return px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;

    case VelocityFrame::LocalNED: return px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

    case VelocityFrame::LocalFRD: return px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD;

    case VelocityFrame::BodyFRD: return px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
  }

  return px4_msgs::msg::VehicleOdometry::POSE_FRAME_UNKNOWN;
}

} // namespace px4_ros2
