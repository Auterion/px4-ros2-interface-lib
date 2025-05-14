/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "rclcpp/rclcpp.hpp"

#include <mission.hpp>

static const std::string kNodeName = "example_mode_mission";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(kNodeName);

  if (kEnableDebugOutput) {
    auto ret =
      rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(
        node->get_logger(), "Error setting severity: %s",
        rcutils_get_error_string().str);
      rcutils_reset_error();
    }
  }

  Mission mission(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
