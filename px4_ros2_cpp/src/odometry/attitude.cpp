/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/odometry/attitude.hpp>

namespace px4_ros2
{

OdometryAttitude::OdometryAttitude(Context & context)
: Subscription<px4_msgs::msg::VehicleAttitude>(context, "fmu/out/vehicle_attitude")
{
  RequirementFlags requirements{};
  requirements.attitude = true;
  context.setRequirement(requirements);
}

} // namespace px4_ros2
