/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/odometry/sensor_data.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

SensorData::SensorData(Context & context)
: Subscription<px4_msgs::msg::SensorCombined>(context,
    "fmu/out/sensor_combined" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::SensorCombined>())
{
  RequirementFlags requirements{};
  context.setRequirement(requirements);
}

} // namespace px4_ros2
