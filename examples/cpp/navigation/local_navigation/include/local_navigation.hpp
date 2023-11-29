/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once


#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/local_navigation_interface.hpp>

using namespace std::chrono_literals; // NOLINT

class LocalNavigationTest : public px4_ros2::LocalNavigationInterface
{
public:
  LocalNavigationTest(rclcpp::Node & node)
  : LocalNavigationInterface(node, px4_ros2::PoseFrame::NED,
      px4_ros2::VelocityFrame::NED)
  {
    _timer =
      node.create_wall_timer(1s, [this] {updateLocalPosition();});

    RCLCPP_INFO(node.get_logger(), "example_local_navigation_node running!");
  }

  void updateLocalPosition()
  {
    px4_ros2::LocalPositionEstimate local_position_estimate {};

    local_position_estimate.timestamp_sample = _node.get_clock()->now();

    local_position_estimate.velocity_xy = Eigen::Vector2f {1.F, 2.F};
    local_position_estimate.velocity_xy_variance = Eigen::Vector2f {0.3F, 0.4F};

    local_position_estimate.position_z = 12.3F;
    local_position_estimate.position_z_variance = 0.33F;

    local_position_estimate.attitude_quaternion = Eigen::Quaternionf {0.1, -0.2, 0.3, 0.25};
    local_position_estimate.attitude_variance = Eigen::Vector3f {0.2, 0.1, 0.05};

    px4_ros2::NavigationInterfaceReturnCode retcode = update(local_position_estimate);

    RCLCPP_DEBUG_THROTTLE(
      _node.get_logger(),
      *_node.get_clock(), 1000, "Interface returned with: %s.", resultToString(retcode));
  }

private:
  rclcpp::TimerBase::SharedPtr _timer;
};

class ExampleLocalNavigationNode : public rclcpp::Node
{
public:
  ExampleLocalNavigationNode()
  : Node("example_local_navigation_node")
  {
    // Enable debug output
    auto ret =
      rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
    }

    _interface = std::make_unique<LocalNavigationTest>(*this);

    if (!_interface->doRegister()) {
      throw std::runtime_error("Registration failed");
    }
  }

private:
  std::unique_ptr<LocalNavigationTest> _interface;
};
