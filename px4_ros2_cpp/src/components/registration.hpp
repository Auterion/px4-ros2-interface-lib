/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/register_ext_component_request.hpp>
#include <px4_msgs/msg/register_ext_component_reply.hpp>
#include <px4_msgs/msg/unregister_ext_component.hpp>

#include <px4_ros2/components/mode.hpp>

struct RegistrationSettings
{
  std::string name;
  bool register_arming_check{false};
  bool register_mode{false};
  bool register_mode_executor{false};

  bool enable_replace_internal_mode{false};
  px4_ros2::ModeBase::ModeID replace_internal_mode{};
  bool activate_mode_immediately{false};
};

class Registration
{
public:
  explicit Registration(rclcpp::Node & node, const std::string & topic_namespace_prefix = "");
  virtual ~Registration()
  {
    doUnregister();
  }

  virtual bool doRegister(const RegistrationSettings & settings);
  virtual void doUnregister();

  bool registered() const {return _registered;}

  int armingCheckId() const {return _unregister_ext_component.arming_check_id;}
  px4_ros2::ModeBase::ModeID modeId() const {return _unregister_ext_component.mode_id;}
  int modeExecutorId() const {return _unregister_ext_component.mode_executor_id;}

  std::string name() const
  {
    return reinterpret_cast<const char *>(_unregister_ext_component.name.data());
  }

protected:
  void setRegistrationDetails(
    int arming_check_id, px4_ros2::ModeBase::ModeID mode_id,
    int mode_executor_id);

private:
  rclcpp::Subscription<px4_msgs::msg::RegisterExtComponentReply>::SharedPtr
    _register_ext_component_reply_sub;
  rclcpp::Publisher<px4_msgs::msg::RegisterExtComponentRequest>::SharedPtr
    _register_ext_component_request_pub;
  rclcpp::Publisher<px4_msgs::msg::UnregisterExtComponent>::SharedPtr _unregister_ext_component_pub;

  bool _registered{false};
  px4_msgs::msg::UnregisterExtComponent _unregister_ext_component{};
  rclcpp::Node & _node;
};
