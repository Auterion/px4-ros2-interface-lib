/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/odometry/global_position.hpp>

namespace px4_ros2
{

OdometryGlobalPosition::OdometryGlobalPosition(Context & context)
: _node(context.node()),
  _vehicle_global_position_sub(context, "/fmu/out/vehicle_global_position"),
  _last_vehicle_global_position(_node.get_clock()->now() - 1s)
{
  _vehicle_global_position_sub.subscribe(
    [this](const px4_msgs::msg::VehicleGlobalPosition msg) {
      _vehicle_global_position = msg;
      _last_vehicle_global_position = _node.get_clock()->now();
    });

  RequirementFlags requirements{};
  requirements.global_position = true;
  context.setRequirement(requirements);
}

} // namespace px4_ros2
