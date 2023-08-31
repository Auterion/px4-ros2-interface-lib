/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_sdk/odometry/global_position.hpp>


namespace px4_sdk
{

OdometryGlobalPosition::OdometryGlobalPosition(Context & context)
: _node(context.node()),
  _last_vehicle_global_position(_node.get_clock()->now() - 1s)
{
  _vehicle_global_position_sub =
    context.node().create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
    context.topicNamespacePrefix() + "/fmu/out/vehicle_global_position", rclcpp::QoS(
      1).best_effort(),
    [this, context](px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) {
      _vehicle_global_position = *msg;
      _last_vehicle_global_position = _node.get_clock()->now();
    });

  RequirementFlags requirements{};
  requirements.global_position = true;
  context.setRequirement(requirements);
}
} // namespace px4_sdk
