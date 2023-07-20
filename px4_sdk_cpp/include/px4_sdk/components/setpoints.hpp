/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <Eigen/Core>

namespace px4_sdk
{

class ModeBase;

class SetpointSender
{
public:
  enum class SetpointConfigurationResult
  {
    Success = 0,
    Timeout,
    FailureOther
  };

  struct SetpointConfiguration
  {
    // TODO...
    bool manual_enabled{false};
    bool auto_enabled{false};
    bool rates_enabled{true};
    bool attitude_enabled{true};
    bool acceleration_enabled{true};
    bool velocity_enabled{true};
    bool position_enabled{true};
    bool altitude_enabled{true};
    bool control_allocation_enabled{true};
    bool climb_rate_enabled{false};
  };

  SetpointSender(
    rclcpp::Node & node, const ModeBase & mode,
    const std::string & topic_namespace_prefix = "");

  SetpointConfigurationResult configureSetpointsSync(const SetpointConfiguration & config);

  void sendTrajectorySetpoint(const Eigen::Vector3f & velocity);

  // TODO: goto, stop, ...

private:
  rclcpp::Node & _node;
  const ModeBase & _mode;
  SetpointConfiguration _setpoint_configuration{};
  rclcpp::Publisher<px4_msgs::msg::VehicleControlMode>::SharedPtr _config_control_setpoints_pub;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_setpoint_pub;
};

} /* namespace px4_sdk */
