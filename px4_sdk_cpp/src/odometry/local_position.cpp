/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_sdk/odometry/local_position.hpp>


namespace px4_sdk
{

OdometryLocalPosition::OdometryLocalPosition(Context & context)
{
  _vehicle_local_position_sub =
    context.node().create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    context.topicNamespacePrefix() + "/fmu/out/vehicle_local_position", rclcpp::QoS(
      1).best_effort(),
    [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
      _vehicle_local_position = *msg;
    });

  RequirementFlags requirements{};
  requirements.local_position = true;
  requirements.local_alt = true;
  context.setRequirement(requirements);
}
} // namespace px4_sdk
