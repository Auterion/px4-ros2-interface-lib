/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>
using namespace std::chrono_literals; // NOLINT

// Set of all messages used by the library (<topic_name>[, <topic_type>])
#define ALL_PX4_ROS2_MESSAGES \
  {"fmu/in/actuator_motors"}, \
  {"fmu/in/actuator_servos"}, \
  {"fmu/in/arming_check_reply"}, \
  {"fmu/in/aux_global_position", "VehicleGlobalPosition"}, \
  {"fmu/in/config_control_setpoints", "VehicleControlMode"}, \
  {"fmu/in/config_overrides_request", "ConfigOverrides"}, \
  {"fmu/in/goto_setpoint"}, \
  {"fmu/in/mode_completed"}, \
  {"fmu/in/register_ext_component_request"}, \
  {"fmu/in/trajectory_setpoint"}, \
  {"fmu/in/unregister_ext_component"}, \
  {"fmu/in/vehicle_attitude_setpoint"}, \
  {"fmu/in/vehicle_command"}, \
  {"fmu/in/vehicle_command_mode_executor", "VehicleCommand"}, \
  {"fmu/in/vehicle_rates_setpoint"}, \
  {"fmu/in/vehicle_visual_odometry", "VehicleOdometry"}, \
  {"fmu/out/airspeed_validated"}, \
  {"fmu/out/arming_check_request"}, \
  {"fmu/out/battery_status"}, \
  {"fmu/out/home_position"}, \
  {"fmu/out/manual_control_setpoint"}, \
  {"fmu/out/mode_completed"}, \
  {"fmu/out/register_ext_component_reply"}, \
  {"fmu/out/vehicle_attitude"}, \
  {"fmu/out/vehicle_angular_velocity"}, \
  {"fmu/out/vehicle_command_ack"}, \
  {"fmu/out/vehicle_global_position"}, \
  {"fmu/out/vehicle_land_detected"}, \
  {"fmu/out/vehicle_local_position"}, \
  {"fmu/out/vehicle_status"}, \
  {"fmu/out/vtol_vehicle_status"}


namespace px4_ros2
{
/** \ingroup components
 *  @{
 */

struct MessageCompatibilityTopic
{
  std::string topic_name;       ///< e.g. "fmu/out/vehicle_status"
  std::string topic_type{""};       ///< e.g. VehicleStatus. If empty, it's inferred from the topic_name // NOLINT
};

/**
 * Check for a set of messages that the definition matches with the one that PX4 is using.
 * @return true on success
 * @ingroup components
 */
bool messageCompatibilityCheck(
  rclcpp::Node & node, const std::vector<MessageCompatibilityTopic> & messages_to_check,
  const std::string & topic_namespace_prefix = "");

/** @}*/
} // namespace px4_ros2
