/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/components/wait_for_fmu.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

bool waitForFMU(
  rclcpp::Node & node, const rclcpp::Duration & timeout,
  const std::string & topic_namespace_prefix)
{
  RCLCPP_DEBUG(node.get_logger(), "Waiting for FMU...");
  const rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub =
    node.create_subscription<px4_msgs::msg::VehicleStatus>(
    topic_namespace_prefix + "fmu/out/vehicle_status" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(), rclcpp::QoS(
      1).best_effort(),
    [](px4_msgs::msg::VehicleStatus::UniquePtr msg) {});

  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(vehicle_status_sub);

  bool got_message = false;
  auto start_time = node.now();

  while (!got_message) {
    auto now = node.now();

    if (now >= start_time + timeout) {
      break;
    }

    auto wait_ret = wait_set.wait(
      (timeout - (now - start_time)).to_chrono<std::chrono::microseconds>());

    if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
      px4_msgs::msg::VehicleStatus msg;
      rclcpp::MessageInfo info;

      if (vehicle_status_sub->take(msg, info)) {
        got_message = true;

      } else {
        RCLCPP_DEBUG(node.get_logger(), "no VehicleStatus message received");
      }

    } else {
      RCLCPP_DEBUG(node.get_logger(), "timeout while waiting for FMU");
    }
  }

  wait_set.remove_subscription(vehicle_status_sub);
  return got_message;
}

} // namespace px4_ros2
