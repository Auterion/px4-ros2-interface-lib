/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "util.hpp"
#include <gtest/gtest.h>
#include <px4_ros2/utils/message_version.hpp>

std::shared_ptr<rclcpp::Node> initNode()
{
  auto test_node = std::make_shared<rclcpp::Node>("testnode");
  auto ret = rcutils_logging_set_logger_level(
    test_node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

  if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(
      test_node->get_logger(), "Error setting severity: %s",
      rcutils_get_error_string().str);
    rcutils_reset_error();
  }

  return test_node;
}

VehicleState::VehicleState(rclcpp::Node & node, const std::string & topic_namespace_prefix)
: _node(node)
{
  const std::string vehicle_status_topic = topic_namespace_prefix + "fmu/out/vehicle_status" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>();
  _vehicle_status_sub = _node.create_subscription<px4_msgs::msg::VehicleStatus>(
    vehicle_status_topic, rclcpp::QoS(1).best_effort(),
    [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
      if (_on_vehicle_status_update) {
        _on_vehicle_status_update(msg);
      }

      if (_on_mode_set_callback && _waiting_for_nav_state == msg->nav_state) {
        if (++_matching_nav_state_set > 2) {                 // wait a bit longer for the mode to be set, and mode activation triggered
          // clear before executing the callback as it might trigger another one
          const ModeSetCallback cb(std::move(_on_mode_set_callback));
          _on_mode_set_callback = nullptr;
          cb();
        }
      }
    });

  const std::string vehicle_command_topic = topic_namespace_prefix + "fmu/in/vehicle_command" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommand>();
  _vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>(
    vehicle_command_topic, 1);

}

void VehicleState::callbackOnModeSet(
  const VehicleState::ModeSetCallback & callback,
  uint8_t nav_state)
{
  assert(_on_mode_set_callback == nullptr);
  _on_mode_set_callback = callback;
  _waiting_for_nav_state = nav_state;
  _matching_nav_state_set = 0;
}

void VehicleState::sendCommand(
  uint32_t command, float param1, float param2, float param3, float param4,
  float param5, float param6, float param7)
{
  // Send command, don't wait for ack
  px4_msgs::msg::VehicleCommand cmd{};
  cmd.command = command;
  cmd.param1 = param1;
  cmd.param2 = param2;
  cmd.param3 = param3;
  cmd.param4 = param4;
  cmd.param5 = param5;
  cmd.param6 = param6;
  cmd.param7 = param7;
  cmd.timestamp = 0; // Let PX4 set the timestamp
  _vehicle_command_pub->publish(cmd);
}

void VehicleState::setGPSFailure(bool failure)
{
  sendCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_INJECT_FAILURE,
    px4_msgs::msg::VehicleCommand::FAILURE_UNIT_SENSOR_GPS,
    failure ? px4_msgs::msg::VehicleCommand::FAILURE_TYPE_OFF :
    px4_msgs::msg::VehicleCommand::FAILURE_TYPE_OK,
    0);
}

void VehicleState::setForceLowBattery(bool enabled)
{
  sendCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_INJECT_FAILURE,
    px4_msgs::msg::VehicleCommand::FAILURE_UNIT_SYSTEM_BATTERY,
    enabled ? px4_msgs::msg::VehicleCommand::FAILURE_TYPE_OFF :
    px4_msgs::msg::VehicleCommand::FAILURE_TYPE_OK,
    0);
}
