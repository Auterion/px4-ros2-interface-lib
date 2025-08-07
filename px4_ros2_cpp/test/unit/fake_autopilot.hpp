/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/mode_completed.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

class FakeAutopilot
{
public:
  FakeAutopilot(
    const std::shared_ptr<rclcpp::Node> & node,
    const std::string & topic_namespace_prefix)
  : _node(node)
  {
    _vehicle_status_pub = node->create_publisher<px4_msgs::msg::VehicleStatus>(
      topic_namespace_prefix + "fmu/out/vehicle_status" + px4_ros2::getMessageNameVersion<
        px4_msgs::msg::VehicleStatus>(),
      1);
    _mode_completed_pub = node->create_publisher<px4_msgs::msg::ModeCompleted>(
      topic_namespace_prefix + "fmu/out/mode_completed" + px4_ros2::getMessageNameVersion<
        px4_msgs::msg::ModeCompleted>(),
      1);
    _land_detected_pub = node->create_publisher<px4_msgs::msg::VehicleLandDetected>(
      topic_namespace_prefix + "fmu/out/vehicle_land_detected" + px4_ros2::getMessageNameVersion<
        px4_msgs::msg::VehicleLandDetected>(),
      1);
    _vehicle_status.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
    _vehicle_status.nav_state = 0;
    _vehicle_status.executor_in_charge = 0;
    _vehicle_status_pub->publish(_vehicle_status);
  }

  void setArmed(bool armed)
  {
    _vehicle_status.arming_state =
      armed ? px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED : px4_msgs::msg::VehicleStatus::
      ARMING_STATE_DISARMED;
    _vehicle_status_pub->publish(_vehicle_status);
  }
  void setModeAndArm(uint8_t mode_id, uint8_t mode_executor = 0)
  {
    RCLCPP_DEBUG(_node->get_logger(), "FakeAutopilot: setting mode (id=%i)", mode_id);
    _vehicle_status.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    _vehicle_status.nav_state = mode_id;
    _vehicle_status.executor_in_charge = mode_executor;
    _vehicle_status_pub->publish(_vehicle_status);

  }

  void sendModeCompleted()
  {
    px4_msgs::msg::ModeCompleted mode_completed;
    mode_completed.nav_state = _vehicle_status.nav_state;
    mode_completed.result = px4_msgs::msg::ModeCompleted::RESULT_SUCCESS;
    RCLCPP_DEBUG(
      _node->get_logger(), "FakeAutopilot: sending mode completed (id=%i)",
      mode_completed.nav_state);
    _mode_completed_pub->publish(mode_completed);
  }

  void setLanded(bool landed)
  {
    RCLCPP_DEBUG(_node->get_logger(), "FakeAutopilot: setting landed to %i", landed);
    px4_msgs::msg::VehicleLandDetected landed_msg;
    landed_msg.landed = landed;
    _land_detected_pub->publish(landed_msg);
  }

private:
  std::shared_ptr<rclcpp::Node> _node;
  px4_msgs::msg::VehicleStatus _vehicle_status;
  rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_pub;
  rclcpp::Publisher<px4_msgs::msg::ModeCompleted>::SharedPtr _mode_completed_pub;
  rclcpp::Publisher<px4_msgs::msg::VehicleLandDetected>::SharedPtr _land_detected_pub;
};
