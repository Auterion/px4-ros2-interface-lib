/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <px4_ros2/utils/message_version.hpp>

#include <chrono>
#include <thread>

namespace px4_ros2 {

bool waitForFMU(rclcpp::Node& node, const rclcpp::Duration& discovery_timeout,
                const rclcpp::Duration& heartbeat_timeout,
                const std::string& topic_namespace_prefix)
{
  RCLCPP_DEBUG(node.get_logger(), "Waiting for FMU...");
  const std::string topic = topic_namespace_prefix + "fmu/out/vehicle_status" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>();
  const rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub =
      node.create_subscription<px4_msgs::msg::VehicleStatus>(
          topic, rclcpp::QoS(1).best_effort(),
          [](px4_msgs::msg::VehicleStatus::UniquePtr msg) {});

  // Phase 1: wait for the FMU's vehicle_status publisher to appear on the graph.
  using namespace std::chrono_literals;  // NOLINT
  const auto discovery_start = node.now();
  while (node.count_publishers(topic) == 0) {
    if (node.now() >= discovery_start + discovery_timeout) {
      RCLCPP_DEBUG(node.get_logger(), "timeout while waiting for FMU publisher discovery");
      return false;
    }
    std::this_thread::sleep_for(50ms);
  }

  // Phase 2: wait for the first vehicle_status message on the matched subscription.
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(vehicle_status_sub);

  bool got_message = false;
  const auto heartbeat_start = node.now();

  while (!got_message) {
    const auto now = node.now();

    if (now >= heartbeat_start + heartbeat_timeout) {
      break;
    }

    const auto wait_ret = wait_set.wait(
        (heartbeat_timeout - (now - heartbeat_start)).to_chrono<std::chrono::microseconds>());

    if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
      px4_msgs::msg::VehicleStatus msg;
      rclcpp::MessageInfo info;

      if (vehicle_status_sub->take(msg, info)) {
        got_message = true;

      } else {
        RCLCPP_DEBUG(node.get_logger(), "no VehicleStatus message received");
      }

    } else {
      RCLCPP_DEBUG(node.get_logger(), "timeout while waiting for FMU heartbeat");
    }
  }

  wait_set.remove_subscription(vehicle_status_sub);
  return got_message;
}

bool waitForFMU(rclcpp::Node& node, const rclcpp::Duration& timeout,
                const std::string& topic_namespace_prefix)
{
  return waitForFMU(node, timeout, timeout, topic_namespace_prefix);
}

}  // namespace px4_ros2
