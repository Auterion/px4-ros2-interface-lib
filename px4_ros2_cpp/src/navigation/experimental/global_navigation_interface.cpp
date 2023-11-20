/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/navigation/experimental/global_navigation_interface.hpp>


namespace px4_ros2
{

GlobalNavigationInterface::GlobalNavigationInterface(rclcpp::Node & node, uint8_t altitude_frame)
: _node(node),
  _altitude_frame(altitude_frame)
{
  _aux_global_position_pub =
    node.create_publisher<AuxGlobalPosition>(AUX_GLOBAL_POSITION_TOPIC, 10);
}

int GlobalNavigationInterface::update(const GlobalPositionEstimate & global_position_estimate) const
{
  AuxGlobalPosition aux_global_position;

  aux_global_position.timestamp_sample = global_position_estimate.timestamp_sample;

  // Publish
  aux_global_position.timestamp = _node.get_clock()->now().nanoseconds() * 1e-3;
  _aux_global_position_pub->publish(aux_global_position);

  return static_cast<int>(NavigationInterfaceCodes::SUCCESS);
}

} // namespace px4_ros2
