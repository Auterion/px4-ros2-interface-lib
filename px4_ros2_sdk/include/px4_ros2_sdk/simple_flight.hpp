/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <px4_ros2_sdk/result.hpp>
#include <px4_ros2_sdk_interfaces/action/takeoff.hpp>
#include <px4_ros2_sdk_interfaces/srv/set_armed.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

namespace px4_ros2::sdk {
/** \ingroup sdk
 *  @{
 */

/**
 * @brief Ergonomic command facade over a FlightServer's ROS 2 service+action
 *        graph (the "simple commander" tier).
 *
 * SimpleFlight is a thin action-plus-service client: its blocking verbs send a
 * request or goal to the FlightServer and spin the borrowed node until the
 * outcome is known. It is the easy path for scripts and opens no `/fmu`
 * endpoint of its own; its service/action clients are companion-side, and all
 * FMU I/O stays inside the FlightServer (and, beneath it, px4_ros2_cpp).
 *
 * The borrowed node must not be spun concurrently by another executor while a
 * SimpleFlight call is in progress.
 */
class SimpleFlight {
 public:
  using SetArmed = px4_ros2_sdk_interfaces::srv::SetArmed;
  using Takeoff = px4_ros2_sdk_interfaces::action::Takeoff;

  /**
   * @param node             ROS 2 node used for the service/action clients.
   * @param server_namespace namespace the target FlightServer runs in (e.g.
   *                         "/vehicle_1"; empty for the default namespace).
   */
  explicit SimpleFlight(rclcpp::Node::SharedPtr node, const std::string& server_namespace = "");

  /// Arm the vehicle.
  Result arm(bool force = false, std::chrono::milliseconds timeout = std::chrono::seconds(5));
  /// Disarm the vehicle.
  Result disarm(std::chrono::milliseconds timeout = std::chrono::seconds(5));
  /// Take off to a target altitude (NaN uses PX4's default), blocking on the action result.
  Result takeoff(float altitude_amsl_m = NAN,
                 std::chrono::milliseconds timeout = std::chrono::seconds(60));

 private:
  Result setArmed(bool arm, bool force, std::chrono::milliseconds timeout);

  rclcpp::Node::SharedPtr _node;
  rclcpp::Client<SetArmed>::SharedPtr _arm_client;
  rclcpp_action::Client<Takeoff>::SharedPtr _takeoff_client;
};

/** @}*/
}  // namespace px4_ros2::sdk
