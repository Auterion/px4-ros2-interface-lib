/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once


#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/global_position_measurement_interface.hpp>

using namespace std::chrono_literals; // NOLINT

class GlobalNavigationTest : public px4_ros2::GlobalPositionMeasurementInterface
{
public:
  explicit GlobalNavigationTest(rclcpp::Node & node)
  : GlobalPositionMeasurementInterface(node)
  {
    _timer =
      node.create_wall_timer(1s, [this] {updateGlobalPosition();});

    RCLCPP_INFO(node.get_logger(), "example_global_navigation_node running!");
  }

  void updateGlobalPosition()
  {
    px4_ros2::GlobalPositionMeasurement global_position_measurement {};

    global_position_measurement.timestamp_sample = _node.get_clock()->now();

    global_position_measurement.lat_lon = Eigen::Vector2d {12.34321, 23.45432};
    global_position_measurement.horizontal_variance = 0.1f;

    global_position_measurement.altitude_msl = 12.4f;
    global_position_measurement.vertical_variance = 0.2f;

    try {
      update(global_position_measurement);
      RCLCPP_DEBUG(
        _node.get_logger(),
        "Successfully sent position update to navigation interface.");
    } catch (const px4_ros2::NavigationInterfaceInvalidArgument & e) {
      RCLCPP_ERROR_THROTTLE(
        _node.get_logger(),
        *_node.get_clock(), 1000, "Exception caught: %s", e.what());
    }
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

    if (!_interface->doRegister()) {
      throw std::runtime_error("Registration failed");
    }
  }

private:
  std::unique_ptr<GlobalNavigationTest> _interface;
};
