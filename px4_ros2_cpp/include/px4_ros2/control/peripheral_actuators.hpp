/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/vehicle_command.hpp>
#include <Eigen/Core>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup control
 *  @{
 */

/**
 * @brief Provides control of one or more extra actuators.
 *
 * It maps to the 'Peripheral Actuator Set' output functions on the PX4 side.
 * This can be used by a mode independent from the setpoints.
 */
class PeripheralActuatorControls
{
public:
  static constexpr int kNumActuators = 6;

  explicit PeripheralActuatorControls(Context & context);

  /**
   * Control actuators
   * @param values range [-1, 1], NAN=ignore value
   */
  void set(const Eigen::Matrix<float, kNumActuators, 1> & values);

  /**
   * Sets a single actuator output
   * @param value range [-1, 1]
   * @param index range [0, kNumActuators)
   */
  void set(float value, unsigned index = 0);

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
  rclcpp::Time _last_update{};
};

/** @}*/
} /* namespace px4_ros2 */
