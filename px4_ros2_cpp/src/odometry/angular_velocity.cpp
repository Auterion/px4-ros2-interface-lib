/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/odometry/angular_velocity.hpp>

namespace px4_ros2
{

OdometryAngularVelocity::OdometryAngularVelocity(Context & context)
: Subscription<px4_msgs::msg::VehicleAngularVelocity>(context, "fmu/out/vehicle_angular_velocity")
{
  RequirementFlags requirements{};
  requirements.angular_velocity = true;
  context.setRequirement(requirements);
}

} // namespace px4_ros2
