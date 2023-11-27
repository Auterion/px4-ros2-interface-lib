/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/rover_cruise_control_state.hpp>
#include <px4_msgs/msg/rover_cruise_control_velocity.hpp>
#include <Eigen/Core>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{

class CruiseControlSetpointType : public SetpointBase
{
public:
  explicit CruiseControlSetpointType(Context & context);

  ~CruiseControlSetpointType() override = default;

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override {return 100.F;}

  void update(float linear_velocity_x, float angular_velocity_z);
  px4_msgs::msg::RoverCruiseControlState initialize_state();

  float _max_linear_velocity_x{0.0f};
  float _max_angular_velocity_z{0.0f};
  float _cruise_control_gain{0.0f};
  float _linear_velocity_x{0.0f};
  float _angular_velocity_z{0.0f};

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::RoverCruiseControlVelocity>::SharedPtr
    _cruise_control_velocity_pub;
  rclcpp::Subscription<px4_msgs::msg::RoverCruiseControlState>::SharedPtr _rover_cruise_control;
};

} /* namespace px4_ros2 */
