/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

 #include <px4_ros2/control/vtol.hpp>
 #include <px4_ros2/utils/message_version.hpp>
 #include <rclcpp/rclcpp.hpp>

 
 using namespace std::chrono_literals;
 
 namespace px4_ros2
 {
 
 VTOL::VTOL(Context & context)
 : _node(context.node())
 {
    _vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>(
    context.topicNamespacePrefix() + "fmu/in/vehicle_command" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommand>(),
    1);
    _vehicle_status_sub  = _node.create_subscription<px4_msgs::msg::VehicleStatus>(
    context.topicNamespacePrefix() + "fmu/out/vehicle_status" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(),
    rclcpp::QoS(10).best_effort(), 
    [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
        _system_id = msg->system_id; 
        _component_id = msg->component_id; 
        _vehicle_type = msg->vehicle_type;

    });

 }
 
 
 void VTOL::transition()
 {  

    if(_vehicle_type == px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROTARY_WING){

        px4_msgs::msg::VehicleCommand cmd;
        
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION; 
        cmd.param1 = px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_FW;

            cmd.param2 = 0;
    
            cmd.target_system = _system_id; 
            cmd.target_component = _component_id; 
    
            cmd.timestamp = 0; // Let PX4 set the timestamp
    
            _vehicle_command_pub->publish(cmd);

    }
    else if(_vehicle_type == px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_FIXED_WING){


        px4_msgs::msg::VehicleCommand cmd;
        
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION; 
        cmd.param1 = px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_MC;

            cmd.param2 = 0;
    
            cmd.target_system = _system_id; 
            cmd.target_component = _component_id; 
    
            cmd.timestamp = 0; // Let PX4 set the timestamp
    
            _vehicle_command_pub->publish(cmd);

    }
   
 }
 
 } // namespace px4_ros2
 