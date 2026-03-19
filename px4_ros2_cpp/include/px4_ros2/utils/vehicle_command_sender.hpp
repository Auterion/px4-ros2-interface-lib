/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace px4_ros2 {
/** \ingroup utils
 *  @{
 */

/**
 * @brief Sends a VehicleCommand and waits synchronously for an ACK from PX4.
 *
 * This helper can be reused by any component that needs to send one-shot
 * vehicle commands with acknowledgement (e.g. HomePositionSetter,
 * ModeExecutorBase).
 */
class VehicleCommandSender {
 public:
  /**
   * @param node          ROS 2 node used for creating publishers/subscriptions
   * @param topic_namespace_prefix  topic namespace (e.g. "" or "/my_ns/")
   * @param command_topic command topic name without prefix/version suffix
   *                      (default: "fmu/in/vehicle_command")
   */
  explicit VehicleCommandSender(rclcpp::Node& node, const std::string& topic_namespace_prefix,
                                const std::string& command_topic = "fmu/in/vehicle_command");

  /**
   * @brief Publish @p cmd and wait for a matching VehicleCommandAck.
   *
   * Retries up to 3 times with a 300 ms timeout per attempt.
   * The ACK is matched on (command, target_component == cmd.source_component).
   *
   * @return Result::Success on ACK accepted, Result::Rejected on ACK
   *         rejected, Result::Timeout if no ACK is received.
   */
  Result sendCommandSync(px4_msgs::msg::VehicleCommand cmd);

 private:
  rclcpp::Node& _node;
  std::string _topic_namespace_prefix;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
};

/** @}*/
}  // namespace px4_ros2
