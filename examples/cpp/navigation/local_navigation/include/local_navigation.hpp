/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once


#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/navigation/experimental/local_navigation_interface.hpp>

#include <Eigen/Core>

using namespace std::chrono_literals; // NOLINT

static const std::string kName = "local_navigation_test";
static const std::string kNodeName = "example_local_navigation_node";

class LocalNavigationTest : public px4_ros2::Context
{
public:
  explicit LocalNavigationTest(rclcpp::Node & node)
  : Context(node), _node(node)
  {
    const uint8_t pose_frame = 0;
    const uint8_t velocity_frame = 0;

    _local_navigation_interface = std::make_shared<px4_ros2::LocalNavigationInterface>(
      *this,
      pose_frame,
      velocity_frame);

  }

  void updateAuxLocalPosition()
  {
    px4_ros2::LocalPositionEstimate local_position_estimate {};

    local_position_estimate.timestamp_sample = _node.get_clock()->now().nanoseconds() * 1e-3;
    local_position_estimate.velocity_xy = Eigen::Vector2f{1.f, 2.f};
    local_position_estimate.velocity_xy_variance = Eigen::Vector2f{0.3f, 0.4f};

    _local_navigation_interface->update(local_position_estimate);
  }

private:
  rclcpp::Node & _node;
  std::shared_ptr<px4_ros2::LocalNavigationInterface> _local_navigation_interface;
};

class TestNode : public rclcpp::Node
{
public:
  TestNode()
  : Node(kNodeName)
  {
    // Enable debug output
    auto ret =
      rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
    }

    _local_navigation_test = std::make_unique<LocalNavigationTest>(*this);

    RCLCPP_INFO(get_logger(), "TestNode running!");

    int counter = 10;
    while (counter-- > 0) {
      _local_navigation_test->updateAuxLocalPosition();
      sleep(1);
    }

  }

private:
  std::unique_ptr<LocalNavigationTest> _local_navigation_test;
};
