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

class GlobalNavigationTest : public px4_ros2::Context
{
public:
  explicit GlobalNavigationTest(rclcpp::Node & node)
  : Context(node), _node(node)
  {
    // Instantiate global navigation interface
    _global_navigation_interface = std::make_shared<px4_ros2::GlobalNavigationInterface>(*this);

    _timer =
      node.create_wall_timer(1s, std::bind(&GlobalNavigationTest::updateAuxGlobalPosition, this));

    RCLCPP_INFO(node.get_logger(), "example_global_navigation_node running!");
  }

  void updateAuxGlobalPosition()
  {
    px4_ros2::GlobalPositionEstimate global_position_estimate {};

    global_position_estimate.timestamp_sample = _node.get_clock()->now();

    global_position_estimate.lat_lon = Eigen::Vector2f {12.34321, 23.45432};
    global_position_estimate.altitude_agl = 12.4f;
    global_position_estimate.positional_uncertainty = 0.4f;

    NavigationInterfaceReturnCode retcode;
    retcode = _global_navigation_interface->update(global_position_estimate);

    switch (retcode) {
      case NavigationInterfaceReturnCode::SUCCESS:
        RCLCPP_DEBUG(_node.get_logger(), "Interface returned with: success.");
        break;
      case NavigationInterfaceReturnCode::ESTIMATE_EMPTY:
        RCLCPP_DEBUG(_node.get_logger(), "Interface returned with: estimate empty.");
        break;
      case NavigationInterfaceReturnCode::ESTIMATE_VARIANCE_INVALID:
        RCLCPP_DEBUG(_node.get_logger(), "Interface returned with: variance invalid.");
        break;
      case NavigationInterfaceReturnCode::ESTIMATE_FRAME_UNKNOWN:
        RCLCPP_DEBUG(_node.get_logger(), "Interface returned with: estimate has unknown frame.");
        break;
      case NavigationInterfaceReturnCode::ESTIMATE_VALUE_NAN:
        RCLCPP_DEBUG(_node.get_logger(), "Interface returned with: estimate contains NAN.");
        break;
      case NavigationInterfaceReturnCode::ESTIMATE_MISSING_TIMESTAMP:
        RCLCPP_DEBUG(_node.get_logger(), "Interface returned with: estimate missing timestamp.");
        break;
      default:
        RCLCPP_DEBUG(
          _node.get_logger(), "Interface returned with unknown return code: %i", (int)retcode);
        break;
    }
  }

private:
  std::shared_ptr<px4_ros2::GlobalNavigationInterface> _global_navigation_interface;
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Node & _node;
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
