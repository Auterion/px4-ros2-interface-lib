/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once
#include <cstdint>
#include <memory>
#include <px4_msgs/msg/mode_completed.hpp>
#include <px4_msgs/msg/setpoint_config.hpp>
#include <px4_msgs/msg/setpoint_config_reply.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

/**
 * Records the setpoint configurations that modes apply and, while enabled (the default), replies to
 * them like the FMU does
 */
class SetpointConfigResponder {
 public:
  SetpointConfigResponder(const std::shared_ptr<rclcpp::Node>& node,
                          const std::string& topic_namespace_prefix)
  {
    _reply_pub = node->create_publisher<px4_msgs::msg::SetpointConfigReply>(
        topic_namespace_prefix + "fmu/out/setpoint_config_reply" +
            px4_ros2::getMessageNameVersion<px4_msgs::msg::SetpointConfigReply>(),
        1);
    _config_sub = node->create_subscription<px4_msgs::msg::SetpointConfig>(
        topic_namespace_prefix + "fmu/in/setpoint_config" +
            px4_ros2::getMessageNameVersion<px4_msgs::msg::SetpointConfig>(),
        rclcpp::QoS(10), [this](px4_msgs::msg::SetpointConfig::UniquePtr msg) {
          if (!msg->should_apply) {
            return;
          }
          configs.push_back(*msg);
          if (reply) {
            sendReply(msg->source_id, msg->type);
          }
        });
  }

  // The subscription callback refers to this object
  SetpointConfigResponder(const SetpointConfigResponder&) = delete;
  SetpointConfigResponder& operator=(const SetpointConfigResponder&) = delete;

  void sendReply(uint8_t source_id, uint16_t type)
  {
    px4_msgs::msg::SetpointConfigReply setpoint_config_reply{};
    setpoint_config_reply.source_id = source_id;
    setpoint_config_reply.type = type;
    setpoint_config_reply.result = px4_msgs::msg::SetpointConfigReply::RESULT_SUCCESS;
    _reply_pub->publish(setpoint_config_reply);
  }

  std::vector<px4_msgs::msg::SetpointConfig> configs;
  bool reply{true};

 private:
  rclcpp::Publisher<px4_msgs::msg::SetpointConfigReply>::SharedPtr _reply_pub;
  rclcpp::Subscription<px4_msgs::msg::SetpointConfig>::SharedPtr _config_sub;
};

class FakeAutopilot {
 public:
  FakeAutopilot(const std::shared_ptr<rclcpp::Node>& node,
                const std::string& topic_namespace_prefix)
      : _node(node), _setpoint_config_responder(node, topic_namespace_prefix)
  {
    _vehicle_status_pub = node->create_publisher<px4_msgs::msg::VehicleStatus>(
        topic_namespace_prefix + "fmu/out/vehicle_status" +
            px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(),
        1);
    _mode_completed_pub = node->create_publisher<px4_msgs::msg::ModeCompleted>(
        topic_namespace_prefix + "fmu/out/mode_completed" +
            px4_ros2::getMessageNameVersion<px4_msgs::msg::ModeCompleted>(),
        1);
    _land_detected_pub = node->create_publisher<px4_msgs::msg::VehicleLandDetected>(
        topic_namespace_prefix + "fmu/out/vehicle_land_detected" +
            px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLandDetected>(),
        1);
    _vehicle_status.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
    _vehicle_status.nav_state = 0;
    _vehicle_status.executor_in_charge = 0;
    _vehicle_status_pub->publish(_vehicle_status);
  }

  void setArmed(bool armed)
  {
    _vehicle_status.arming_state = armed ? px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED
                                         : px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
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
    RCLCPP_DEBUG(_node->get_logger(), "FakeAutopilot: sending mode completed (id=%i)",
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

  SetpointConfigResponder& setpointConfigResponder() { return _setpoint_config_responder; }

 private:
  std::shared_ptr<rclcpp::Node> _node;
  px4_msgs::msg::VehicleStatus _vehicle_status;
  rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_pub;
  rclcpp::Publisher<px4_msgs::msg::ModeCompleted>::SharedPtr _mode_completed_pub;
  rclcpp::Publisher<px4_msgs::msg::VehicleLandDetected>::SharedPtr _land_detected_pub;
  SetpointConfigResponder _setpoint_config_responder;
};
