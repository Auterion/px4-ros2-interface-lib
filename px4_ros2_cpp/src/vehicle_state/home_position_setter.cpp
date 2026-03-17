/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/vehicle_state/home_position_setter.hpp>

namespace px4_ros2 {

HomePositionSetter::HomePositionSetter(Context& context) : _node(context.node())
{
  _vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>(
      context.topicNamespacePrefix() + "fmu/in/vehicle_command" +
          px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommand>(),
      1);
}

void HomePositionSetter::setHomeToCurrentPosition()
{
  sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 1.f);
}

void HomePositionSetter::setHome(double latitude, double longitude, float altitude)
{
  sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 0.f, 0.f, 0.f, 0.f,
              latitude, longitude, altitude);
}

void HomePositionSetter::setGpsGlobalOrigin(double latitude, double longitude, float altitude)
{
  sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN, 0.f, 0.f, 0.f,
              0.f, latitude, longitude, altitude);
}

void HomePositionSetter::sendCommand(uint32_t command, float param1, float param2, float param3,
                                     float param4, double param5, double param6, float param7)
{
  px4_msgs::msg::VehicleCommand cmd{};
  cmd.command = command;
  cmd.param1 = param1;
  cmd.param2 = param2;
  cmd.param3 = param3;
  cmd.param4 = param4;
  cmd.param5 = param5;
  cmd.param6 = param6;
  cmd.param7 = param7;
  cmd.target_system = 0;
  cmd.target_component = 1;
  cmd.timestamp = 0;  // Let PX4 set the timestamp
  _vehicle_command_pub->publish(cmd);
}

}  // namespace px4_ros2
