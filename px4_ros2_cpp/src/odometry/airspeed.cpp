/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/odometry/airspeed.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

OdometryAirspeed::OdometryAirspeed(Context & context)
: Subscription<px4_msgs::msg::AirspeedValidated>(context,
    "fmu/out/airspeed_validated" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::AirspeedValidated>())
{}

} // namespace px4_ros2
