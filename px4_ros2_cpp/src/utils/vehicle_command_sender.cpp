/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/utils/vehicle_command_sender.hpp>

using namespace std::chrono_literals;

namespace px4_ros2 {

VehicleCommandSender::VehicleCommandSender(rclcpp::Node& node,
                                           std::string topic_namespace_prefix,
                                           const std::string& command_topic)
    : _node(node), _topic_namespace_prefix(std::move(topic_namespace_prefix))
{
  _vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>(
      _topic_namespace_prefix + command_topic +
          px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommand>(),
      1);
}

Result VehicleCommandSender::sendCommandSync(px4_msgs::msg::VehicleCommand cmd)
{
  Result result{Result::Rejected};
  cmd.timestamp = 0;  // Let PX4 set the timestamp

  // Create a fresh subscription each call to avoid ROS Jazzy WaitSet conflicts
  const auto vehicle_command_ack_sub = _node.create_subscription<px4_msgs::msg::VehicleCommandAck>(
      _topic_namespace_prefix + "fmu/out/vehicle_command_ack" +
          px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommandAck>(),
      rclcpp::QoS(1).best_effort(), [](px4_msgs::msg::VehicleCommandAck::UniquePtr) {});

  // Wait until we have a publisher on the ACK topic
  auto start_time = std::chrono::steady_clock::now();
  while (vehicle_command_ack_sub->get_publisher_count() == 0) {
    const auto timeout = 3000ms;
    const auto now = std::chrono::steady_clock::now();
    if (now >= start_time + timeout) {
      RCLCPP_WARN(_node.get_logger(), "Timeout waiting for vehicle_command_ack publisher");
      break;
    }
  }

  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(vehicle_command_ack_sub);

  bool got_reply = false;

  for (int retries = 0; retries < 3 && !got_reply; ++retries) {
    _vehicle_command_pub->publish(cmd);
    start_time = std::chrono::steady_clock::now();
    const auto timeout = 300ms;
    while (!got_reply) {
      auto now = std::chrono::steady_clock::now();

      if (now >= start_time + timeout) {
        break;
      }

      auto wait_ret = wait_set.wait(timeout - (now - start_time));

      if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
        px4_msgs::msg::VehicleCommandAck ack;
        rclcpp::MessageInfo info;

        if (vehicle_command_ack_sub->take(ack, info)) {
          if (ack.command == cmd.command && ack.target_component == cmd.source_component) {
            if (ack.result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
              result = Result::Success;
            }

            got_reply = true;
          }

        } else {
          RCLCPP_DEBUG(_node.get_logger(), "No VehicleCommandAck message received");
        }

      } else {
        RCLCPP_DEBUG(_node.get_logger(), "timeout");
      }
    }
  }

  wait_set.remove_subscription(vehicle_command_ack_sub);

  if (!got_reply) {
    result = Result::Timeout;
    RCLCPP_WARN(_node.get_logger(), "Cmd %i: timeout, no ack received", cmd.command);
  }

  return result;
}

}  // namespace px4_ros2
