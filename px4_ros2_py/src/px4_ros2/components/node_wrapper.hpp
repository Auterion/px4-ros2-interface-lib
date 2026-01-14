/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <pybind11/pybind11.h>

#include <rclcpp/rclcpp.hpp>

/**
 * \brief Contains a `rclcpp::Node` and executes it (optionally in a separate thread)
 *
 * This is required as rclpy is directly based on the rcl C library, and there
 * is no way to create/convert a rclcpp::Node from rclpy or vice versa.
 * See also https://github.com/ros2/rclpy/issues/291.
 */
class NodeWrapper {
 public:
  NodeWrapper(const std::string& node_name, bool debug_output);
  ~NodeWrapper();
  const std::shared_ptr<rclcpp::Node>& node() const { return _node; }

  void spin();
  void spinNonBlocking();

 private:
  std::shared_ptr<rclcpp::Node> _node;
  std::thread _thread;
};

void bindNodeWrapper(pybind11::module& m);
