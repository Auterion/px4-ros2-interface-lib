/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "px4_sdk/components/setpoints.hpp"
#include "px4_sdk/components/mode.hpp"

#include <cassert>

namespace px4_sdk
{

SetpointSender::SetpointSender(
  rclcpp::Node & node, const ModeBase & mode,
  const std::string & topic_namespace_prefix)
: _node(node), _mode(mode)
{
  _config_control_setpoints_pub = node.create_publisher<px4_msgs::msg::VehicleControlMode>(
    topic_namespace_prefix + "/fmu/in/config_control_setpoints", 1);

  _trajectory_setpoint_pub = node.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    topic_namespace_prefix + "/fmu/in/trajectory_setpoint", 1);
}

SetpointSender::SetpointConfigurationResult SetpointSender::configureSetpointsSync(
  const SetpointConfiguration & config)
{
  assert(_mode.id() != ModeBase::kModeIDInvalid);       // Ensure mode is registered

  px4_msgs::msg::VehicleControlMode control_mode{};
  control_mode.source_id = static_cast<uint8_t>(_mode.id());
  control_mode.flag_control_manual_enabled = config.manual_enabled;
  control_mode.flag_control_auto_enabled = config.auto_enabled;
  control_mode.flag_control_rates_enabled = config.rates_enabled;
  control_mode.flag_control_attitude_enabled = config.attitude_enabled;
  control_mode.flag_control_acceleration_enabled = config.acceleration_enabled;
  control_mode.flag_control_velocity_enabled = config.velocity_enabled;
  control_mode.flag_control_position_enabled = config.position_enabled;
  control_mode.flag_control_altitude_enabled = config.altitude_enabled;
  control_mode.flag_control_allocation_enabled = config.control_allocation_enabled;
  control_mode.flag_control_climb_rate_enabled = config.climb_rate_enabled;
  control_mode.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
  _config_control_setpoints_pub->publish(control_mode);
  // TODO: wait for feedback from FMU

  _setpoint_configuration = config;
  return SetpointConfigurationResult::Success;
}

void SetpointSender::sendTrajectorySetpoint(const Eigen::Vector3f & velocity)
{
  // TODO: check if configured setpoints match

  // TODO: add mode id to setpoint

  px4_msgs::msg::TrajectorySetpoint sp{};
  sp.yaw = NAN;
  sp.yawspeed = NAN;
  sp.position[0] = sp.position[1] = sp.position[2] = NAN;       // TODO ...
  sp.acceleration[0] = sp.acceleration[1] = sp.acceleration[2] = NAN;
  sp.velocity[0] = velocity(0);
  sp.velocity[1] = velocity(1);
  sp.velocity[2] = velocity(2);
  sp.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
  _trajectory_setpoint_pub->publish(sp);
}

} // namespace px4_sdk
