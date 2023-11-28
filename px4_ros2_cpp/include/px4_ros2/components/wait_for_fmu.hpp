/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>
using namespace std::chrono_literals; // NOLINT

namespace px4_ros2
{

/** \ingroup components
 *  @{
 */
/**
 * Wait for a heartbeat/status message from the FMU
 * @return true on success
 */
bool waitForFMU(
  rclcpp::Node & node, const rclcpp::Duration & timeout = 30s,
  const std::string & topic_namespace_prefix = "");

/** @}*/
} // namespace px4_ros2
