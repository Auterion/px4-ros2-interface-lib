/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/vehicle_state/home_position_setter.hpp>

namespace px4_ros2 {

HomePositionSetter::HomePositionSetter(Context& context)
    : _command_sender(context.node(), context.topicNamespacePrefix())
{
}

Result HomePositionSetter::setHomeToCurrentPosition()
{
  return sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 1.f);
}

Result HomePositionSetter::setHome(double latitude, double longitude, float altitude)
{
  return sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 0.f, 0.f, 0.f, 0.f,
                     latitude, longitude, altitude);
}

Result HomePositionSetter::setGpsGlobalOrigin(double latitude, double longitude, float altitude)
{
  return sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN, 0.f, 0.f,
                     0.f, 0.f, latitude, longitude, altitude);
}

Result HomePositionSetter::sendCommand(uint32_t command, float param1, float param2, float param3,
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
  return _command_sender.sendCommandSync(cmd);
}

}  // namespace px4_ros2
