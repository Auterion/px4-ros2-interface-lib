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
  {"fmu/in/fixed_wing_lateral_setpoint"}, \
  {"fmu/in/fixed_wing_longitudinal_setpoint"}, \
  {"fmu/in/goto_setpoint"}, \
  {"fmu/in/lateral_control_configuration"}, \
  {"fmu/in/longitudinal_control_configuration"}, \
  {"fmu/in/mode_completed"}, \
  {"fmu/in/register_ext_component_request"}, \
  {"fmu/in/rover_attitude_setpoint"}, \
  {"fmu/in/rover_position_setpoint"}, \
  {"fmu/in/rover_rate_setpoint"}, \
  {"fmu/in/rover_speed_setpoint"}, \
  {"fmu/in/rover_steering_setpoint"}, \
  {"fmu/in/rover_throttle_setpoint"}, \
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
 * @brief Check for a set of messages that match the definitions used by PX4.
 *
 * This function verifies that the message definitions are compatible with those
 * expected by the PX4 ROS 2 interface library. It returns true if all definitions match.
 *
 * @note Compared to rmw_fastrtps_cpp, rmw_zenoh_cpp behaves differently.
 * PX4 Zenoh allows flexible topic name mapping, so topic names are not fixed.
 * In this case, the function only verifies the hashes of the message types
 * to ensure binary compatibility between the FMU and ROS 2.
 *
 * ROS 2 Humble does not support type hash, so no type checking will occur
 * when using Zenoh. It is recommended to use Zenoh with ROS 2 Jazzy or later
 * to ensure proper type compatibility verification.
 *
 * @return true on success
 * @ingroup components
 */
bool messageCompatibilityCheck(
  rclcpp::Node & node, const std::vector<MessageCompatibilityTopic> & messages_to_check,
  const std::string & topic_namespace_prefix = "");

/** @}*/
} // namespace px4_ros2
