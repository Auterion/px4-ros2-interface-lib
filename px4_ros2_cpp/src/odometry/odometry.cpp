/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/odometry/odometry.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2 {

Odometry::Odometry(Context& context)
    : Subscription<px4_msgs::msg::VehicleOdometry>(
          context, "fmu/out/vehicle_odometry" +
                       px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleOdometry>())
{
  RequirementFlags requirements{};
  requirements.local_position = true;
  requirements.local_alt = true;
  requirements.attitude = true;
  requirements.angular_velocity = true;

  context.setRequirement(requirements);
}

}  // namespace px4_ros2
