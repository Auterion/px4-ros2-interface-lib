/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/navigation/experimental/global_navigation_interface.hpp>


namespace px4_ros2
{

GlobalNavigationInterface::GlobalNavigationInterface(uint8_t altitude_frame)
: Node("global_navigation_interface_node"),
  _aux_global_position_pub(create_publisher<AuxGlobalPosition>(AUX_GLOBAL_POSITION_TOPIC, 10)),
  _altitude_frame(altitude_frame) {}

void GlobalNavigationInterface::update(GlobalPositionEstimate & global_position_estimate)
{
  AuxGlobalPosition aux_global_position;

  aux_global_position.timestamp_sample = global_position_estimate.timestamp_sample;

  // Publish
  aux_global_position.timestamp = this->now().nanoseconds() * 1e-3;
  _aux_global_position_pub->publish(aux_global_position);
}

} // namespace px4_ros2
