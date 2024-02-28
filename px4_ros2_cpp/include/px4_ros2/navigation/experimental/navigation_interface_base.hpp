/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_ros2/common/context.hpp>

namespace px4_ros2
{

/**
 * @brief Thrown to report invalid arguments to measurement interface
*/
class NavigationInterfaceInvalidArgument : public std::invalid_argument
{
public:
  explicit NavigationInterfaceInvalidArgument(const std::string & message)
  : std::invalid_argument("PX4 ROS2 navigation interface: invalid argument: " + message) {}
};

/**
 * @brief Base class for position measurement interface
*/
class PositionMeasurementInterfaceBase : public Context
{
public:
  explicit PositionMeasurementInterfaceBase(
    rclcpp::Node & node,
    std::string topic_namespace_prefix = "")
  : Context(node, std::move(topic_namespace_prefix)), _node(node) {}
  virtual ~PositionMeasurementInterfaceBase() = default;

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
