/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

OdometryLocalPosition::OdometryLocalPosition(Context & context)
: Subscription<px4_msgs::msg::VehicleLocalPosition>(context,
    "fmu/out/vehicle_local_position" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>())
{
  RequirementFlags requirements{};
  requirements.local_position = true;
  requirements.local_alt = true;
  context.setRequirement(requirements);
}

} // namespace px4_ros2
