/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

OdometryGlobalPosition::OdometryGlobalPosition(Context & context)
: Subscription<px4_msgs::msg::VehicleGlobalPosition>(context,
    "fmu/out/vehicle_global_position" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleGlobalPosition>())
{
  RequirementFlags requirements{};
  requirements.global_position = true;
  context.setRequirement(requirements);
}

} // namespace px4_ros2
