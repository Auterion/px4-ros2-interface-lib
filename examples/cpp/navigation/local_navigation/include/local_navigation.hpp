/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once


#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/local_navigation_interface.hpp>

#include <Eigen/Core>

using namespace std::chrono_literals; // NOLINT

class LocalNavigationTest : public px4_ros2::LocalNavigationInterface
{
public:
  LocalNavigationTest(
    rclcpp::Node & node, const uint8_t pose_frame,
    const uint8_t velocity_frame)
  : LocalNavigationInterface(node, pose_frame, velocity_frame)
  {
    _timer =
      node.create_wall_timer(1s, std::bind(&LocalNavigationTest::updateAuxLocalPosition, this));

    RCLCPP_INFO(node.get_logger(), "example_local_navigation_node running!");
  }

  void updateAuxLocalPosition()
  {
    px4_ros2::LocalPositionEstimate local_position_estimate {};

    local_position_estimate.timestamp_sample = _node.get_clock()->now();

    local_position_estimate.velocity_xy = Eigen::Vector2f {1.f, 2.f};
    local_position_estimate.velocity_xy_variance = Eigen::Vector2f {0.3f, 0.4f};

    local_position_estimate.position_z = 12.3f;
    local_position_estimate.position_z_variance = 0.33f;

    local_position_estimate.attitude_quaternion = Eigen::Quaternionf {0.1, -0.2, 0.3, 0.25};
    local_position_estimate.attitude_variance = Eigen::Vector3f {0.2, 0.1, 0.05};

    px4_ros2::NavigationInterfaceReturnCode retcode = update(local_position_estimate);

    RCLCPP_DEBUG(_node.get_logger(), "Interface returned with: %s.", resultToString(retcode));
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

    const uint8_t pose_frame = AuxLocalPosition::POSE_FRAME_NED;
    const uint8_t velocity_frame = AuxLocalPosition::VELOCITY_FRAME_NED;
    _interface = std::make_unique<LocalNavigationTest>(*this, pose_frame, velocity_frame);

  }

private:
  std::unique_ptr<LocalNavigationTest> _interface;
};
