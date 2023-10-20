/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/config_overrides.hpp>

namespace px4_ros2
{

class ModeBase;
class ModeExecutorBase;

class ConfigOverrides
{
public:
  explicit ConfigOverrides(rclcpp::Node & node, const std::string & topic_namespace_prefix = "");

  void controlAutoDisarm(bool enabled);

private:
  void update();

  friend class ModeBase;
  friend class ModeExecutorBase;
  void deferFailsafes(bool enabled, int timeout_s = 0);
  void setup(uint8_t type, uint8_t id);

  rclcpp::Node & _node;

  px4_msgs::msg::ConfigOverrides _current_overrides{};
  rclcpp::Publisher<px4_msgs::msg::ConfigOverrides>::SharedPtr _config_overrides_pub;
  bool _is_setup{false};
  bool _require_update_after_setup{false};
};

} // namespace px4_ros2
