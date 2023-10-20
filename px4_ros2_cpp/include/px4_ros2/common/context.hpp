/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "requirement_flags.hpp"

namespace px4_ros2
{

class SetpointBase;

class Context
{
public:
  explicit Context(rclcpp::Node & node, std::string topic_namespace_prefix = "")
  : _node(node), _topic_namespace_prefix(std::move(topic_namespace_prefix)) {}

  rclcpp::Node & node() {return _node;}
  const std::string & topicNamespacePrefix() const {return _topic_namespace_prefix;}

  virtual void addSetpointType(SetpointBase * setpoint) {}
  virtual void setRequirement(const RequirementFlags & requirement_flags) {}

private:
  rclcpp::Node & _node;
  const std::string _topic_namespace_prefix;
};

} // namespace px4_ros2
