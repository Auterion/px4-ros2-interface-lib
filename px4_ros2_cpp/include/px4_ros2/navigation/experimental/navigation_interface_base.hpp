/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_ros2/common/context.hpp>

namespace px4_ros2
{
class NavigationInterfaceInvalidArgument : public std::invalid_argument
{
public:
  NavigationInterfaceInvalidArgument(const std::string & message)
  : std::invalid_argument("PX4 ROS2 navigation interface: invalid argument: " + message) {}
};

class NavigationInterfaceBase : public Context
{
public:
  explicit NavigationInterfaceBase(rclcpp::Node & node, std::string topic_namespace_prefix = "")
  : Context(node, std::move(topic_namespace_prefix)), _node(node) {}
  virtual ~NavigationInterfaceBase() = default;

  /**
   * Register the interface.
   * @return true on success
   */
  bool doRegister()
  {
    return true;
  }

protected:
  rclcpp::Node & _node;
};

} // namespace px4_ros2
