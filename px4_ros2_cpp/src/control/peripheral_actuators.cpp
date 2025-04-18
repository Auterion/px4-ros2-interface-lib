/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/peripheral_actuators.hpp>
#include <px4_ros2/utils/message_version.hpp>

using namespace std::chrono_literals;

namespace px4_ros2
{

PeripheralActuatorControls::PeripheralActuatorControls(Context & context)
: _node(context.node())
{
  _vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>(
    context.topicNamespacePrefix() + "fmu/in/vehicle_command" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommand>(),
    1);
  _last_update = _node.get_clock()->now();
}

void PeripheralActuatorControls::set(const Eigen::Matrix<float, kNumActuators, 1> & values)
{
  // Rate-limit to avoid spamming the FC with commands at high frequency
  const auto now = _node.get_clock()->now();
  if (now - _last_update > 100ms) {
    _last_update = now;

    // Send command, don't wait for ack
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR;
    static_assert(kNumActuators == 6);
    cmd.param1 = values(0);
    cmd.param2 = values(1);
    cmd.param3 = values(2);
    cmd.param4 = values(3);
    cmd.param5 = values(4);
    cmd.param6 = values(5);
    cmd.param7 = 0; // index
    cmd.timestamp = 0; // Let PX4 set the timestamp
    _vehicle_command_pub->publish(cmd);
  }
}

void PeripheralActuatorControls::set(float value, unsigned int index)
{
  if (std::isnan(value)) {return;}
  Eigen::Matrix<float, kNumActuators, 1> values;
  values.setConstant(NAN);
  if (index < kNumActuators) {
    values(index) = value;
    set(values);
  }
}
} // namespace px4_ros2
