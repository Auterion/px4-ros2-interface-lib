/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once


#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/global_navigation_interface.hpp>
#include <px4_ros2/common/context.hpp>

#include <Eigen/Core>

using namespace std::chrono_literals; // NOLINT

class GlobalNavigationTest : public px4_ros2::GlobalNavigationInterface
{
public:
  explicit GlobalNavigationTest(rclcpp::Node & node)
  : GlobalNavigationInterface(node)
  {
    _timer =
      node.create_wall_timer(1s, [this] {updateAuxGlobalPosition();});

    RCLCPP_INFO(node.get_logger(), "example_global_navigation_node running!");
  }

  void updateAuxGlobalPosition()
  {
    px4_ros2::GlobalPositionEstimate global_position_estimate {};

    global_position_estimate.timestamp_sample = _node.get_clock()->now();

    global_position_estimate.lat_lon = Eigen::Vector2d {12.34321, 23.45432};
    global_position_estimate.horizontal_variance = 0.1F;

    global_position_estimate.altitude_msl = 12.4F;
    global_position_estimate.vertical_variance = 0.2F;

    px4_ros2::NavigationInterfaceReturnCode retcode = update(global_position_estimate);

    RCLCPP_DEBUG(_node.get_logger(), "Interface returned with: %s.", resultToString(retcode));
  }

private:
  rclcpp::TimerBase::SharedPtr _timer;
};

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

    _interface = std::make_unique<GlobalNavigationTest>(*this);
  }

private:
  std::unique_ptr<GlobalNavigationTest> _interface;
};
