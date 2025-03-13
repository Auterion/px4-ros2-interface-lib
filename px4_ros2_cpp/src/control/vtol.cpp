/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

 #include <px4_ros2/control/vtol.hpp>
 #include <px4_ros2/utils/message_version.hpp>
 
 using namespace std::chrono_literals;
 
 namespace px4_ros2
 {
 
 VTOL::VTOL(Context & context)
 : _node(context.node())
 {
   _vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>(
     context.topicNamespacePrefix() + "fmu/in/vehicle_command" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommand>(),
     1);
//    _last_update = _node.get_clock()->now();
 }
 
 void VTOL::transitionFWtoMC()
 {
   // Rate-limit to avoid spamming the FC with commands at high frequency
//    const auto now = _node.get_clock()->now();
//    if (now - _last_update > 100ms) {
    //  _last_update = now;
 
     // Send command, don't wait for ack
     px4_msgs::msg::VehicleCommand cmd;
     cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION; 

     cmd.param1 = 3;
     cmd.param2 = 0;

     cmd.source_system = 1; 
     cmd.target_system = 1; 
     cmd.source_component = 1; 
     cmd.target_component = 1; 

     cmd.timestamp = 0; // Let PX4 set the timestamp

     _vehicle_command_pub->publish(cmd);
   
 }

 void VTOL::transitionMCtoFW()
 {
   // Rate-limit to avoid spamming the FC with commands at high frequency
//    const auto now = _node.get_clock()->now();
//    if (now - _last_update > 100ms) {
    //  _last_update = now;
 
     // Send command, don't wait for ack
     px4_msgs::msg::VehicleCommand cmd;
     cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION; 

     cmd.param1 = 4;
     cmd.param2 = 0;

     cmd.source_system = 1; 
     cmd.target_system = 1; 
     cmd.source_component = 1; 
     cmd.target_component = 1; 

     cmd.timestamp = 0; // Let PX4 set the timestamp
     
     _vehicle_command_pub->publish(cmd);
//    }
 }
 
 } // namespace px4_ros2
 