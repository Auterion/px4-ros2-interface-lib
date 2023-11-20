/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once


#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/global_navigation_interface.hpp>

#include <Eigen/Core>

using namespace std::chrono_literals; // NOLINT

class ExampleGlobalNavigationNode : public rclcpp::Node
{
public:
  ExampleGlobalNavigationNode()
  : Node("example_global_navigation_node")
  {
    // Enable debug output
    auto ret =
      rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
    }

    // Instantiate global navigation interface
    _global_navigation_interface = std::make_shared<px4_ros2::GlobalNavigationInterface>(*this);

    _timer =
      create_wall_timer(1s, std::bind(&ExampleGlobalNavigationNode::updateAuxGlobalPosition, this));

    RCLCPP_INFO(get_logger(), "example_global_navigation_node running!");
  }

  void updateAuxGlobalPosition()
  {
    px4_ros2::GlobalPositionEstimate global_position_estimate {};

    global_position_estimate.timestamp_sample = this->now().nanoseconds() * 1e-3;

    global_position_estimate.position_lat_lon = Eigen::Vector2f {12.34321, 23.45432};
    global_position_estimate.position_variance = 0.4f;

    global_position_estimate.altitude_agl = 12.4f;

    _global_navigation_interface->update(global_position_estimate);
  }

private:
  std::shared_ptr<px4_ros2::GlobalNavigationInterface> _global_navigation_interface;
  rclcpp::TimerBase::SharedPtr _timer;
};
