/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/differential_drive_setpoint.hpp>
#include <Eigen/Core>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{

class ManualRoverControlModeType : public SetpointBase
{
public:
  explicit ManualRoverControlModeType(Context & context);

  ~ManualRoverControlModeType() override = default;

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override {return 100.F;}

  void update(float linear_velocity_x, float angular_velocity_z);
  // px4_msgs::msg::RoverCruiseControlState initialize_state();

private:
  rclcpp::Node & _node;

  rclcpp::Publisher<px4_msgs::msg::DifferentialDriveSetpoint>::SharedPtr _differential_drive_setpoint_pub;
};

} /* namespace px4_ros2 */
