/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals; // NOLINT

/**
 * Wait until a predicate returns true, while spinning a ROS node, with a fixed timeout of 3s
 * @return false on timeout
 */
static inline bool waitFor(
  const std::shared_ptr<rclcpp::Node> & node,
  const std::function<bool()> & predicate)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto start = node->get_clock()->now();

  while (node->get_clock()->now() - start < 3s) {
    if (predicate()) {
      return true;
    }
    executor.spin_some();
    std::this_thread::yield();
  }
  return false;
}
