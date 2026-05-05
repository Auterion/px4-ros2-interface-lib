/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>
using namespace std::chrono_literals;  // NOLINT

namespace px4_ros2 {

/** \ingroup components
 *  @{
 */
/**
 * Wait for the FMU to be discoverable and emit its first heartbeat/status message.
 *
 * The wait happens in two phases:
 *   1. Discovery: poll the ROS graph until the vehicle_status publisher is visible,
 *      bounded by @p discovery_timeout. Covers slow DDS endpoint discovery (e.g.
 *      cold boot).
 *   2. Heartbeat: wait for the first vehicle_status message on the now-matched
 *      subscription, bounded by @p heartbeat_timeout. Should be short — fails
 *      fast when the FMU is discovered but silent.
 *
 * @return true if both phases succeed before their respective timeouts.
 */
bool waitForFMU(rclcpp::Node& node, const rclcpp::Duration& discovery_timeout,
                const rclcpp::Duration& heartbeat_timeout,
                const std::string& topic_namespace_prefix = "");

/**
 * Wait for a heartbeat/status message from the FMU.
 *
 * @deprecated Use the (discovery_timeout, heartbeat_timeout) overload for finer
 *   control over the cold-boot discovery phase versus the heartbeat phase.
 *   This overload calls the new one with both timeouts set to @p timeout, so
 *   the worst-case wait is unchanged.
 * @return true on success
 */
[[deprecated(
    "Use waitForFMU(node, discovery_timeout, heartbeat_timeout, prefix) "
    "for finer control over discovery vs heartbeat timeouts.")]]
bool waitForFMU(rclcpp::Node& node, const rclcpp::Duration& timeout = 30s,
                const std::string& topic_namespace_prefix = "");

/** @}*/
}  // namespace px4_ros2
