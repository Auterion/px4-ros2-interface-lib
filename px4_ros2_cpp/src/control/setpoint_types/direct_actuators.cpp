/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/direct_actuators.hpp>
#include <px4_ros2/utils/message_version.hpp>


namespace px4_ros2
{

DirectActuatorsSetpointType::DirectActuatorsSetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _actuator_motors_pub = context.node().create_publisher<px4_msgs::msg::ActuatorMotors>(
    context.topicNamespacePrefix() + "fmu/in/actuator_motors" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::ActuatorMotors>(),
    1);
  _actuator_servos_pub = context.node().create_publisher<px4_msgs::msg::ActuatorServos>(
    context.topicNamespacePrefix() + "fmu/in/actuator_servos" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::ActuatorServos>(),
    1);
}

void DirectActuatorsSetpointType::updateMotors(
  const Eigen::Matrix<float,
  kMaxNumMotors, 1> & motor_commands)
{
  onUpdate();

  px4_msgs::msg::ActuatorMotors sp_motors{};
  for (int i = 0; i < kMaxNumMotors; ++i) {
    sp_motors.control[i] = motor_commands(i);
  }
  sp_motors.timestamp = 0; // Let PX4 set the timestamp
  _actuator_motors_pub->publish(sp_motors);
}

void DirectActuatorsSetpointType::updateServos(
  const Eigen::Matrix<float,
  kMaxNumServos, 1> & servo_commands)
{
  onUpdate();

  px4_msgs::msg::ActuatorServos sp_servos{};
  for (int i = 0; i < kMaxNumServos; ++i) {
    sp_servos.control[i] = servo_commands(i);
  }
  sp_servos.timestamp = 0; // Let PX4 set the timestamp
  _actuator_servos_pub->publish(sp_servos);
}

SetpointBase::Configuration DirectActuatorsSetpointType::getConfiguration()
{
  Configuration config{};
  config.control_allocation_enabled = false;
  config.rates_enabled = false;
  config.attitude_enabled = false;
  config.altitude_enabled = false;
  config.climb_rate_enabled = false;
  config.acceleration_enabled = false;
  config.velocity_enabled = false;
  config.position_enabled = false;
  return config;
}
} // namespace px4_ros2
