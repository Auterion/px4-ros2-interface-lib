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
        _is_vtol = msg->is_vtol; 

    });

    _vtol_vehicle_status_sub  = _node.create_subscription<px4_msgs::msg::VtolVehicleStatus>(
    context.topicNamespacePrefix() + "fmu/out/vtol_vehicle_status" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VtolVehicleStatus>(),
    rclcpp::QoS(10).best_effort(), 
    [this](px4_msgs::msg::VtolVehicleStatus::UniquePtr msg) {

        switch (msg->vehicle_vtol_state)
        {
        case px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_TRANSITION_TO_FW:
            _current_state = VTOL::State::TRANSITION_TO_FIXED_WING; 
            break;
        case px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_TRANSITION_TO_MC:
            _current_state = VTOL::State::TRANSITION_TO_MULTICOPTER; 
            break; 
        case px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_MC:
            _current_state = VTOL::State::MULTICOPTER; 
            break; 
        case px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_FW:
            _current_state = VTOL::State::FIXED_WING; 
            break; 
        default:
            _current_state = VTOL::State::UNDEFINED; 
        }
        
    });
 }

 VTOL::State VTOL::get_current_state(){
    return _current_state; 
}
 
 void VTOL::transition()
 {  
    if(_is_vtol){
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
    }else{
        RCLCPP_WARN(_node.get_logger(), "VTOL Transition not supported by current vehicle type.");
    }
 }




 
 } // namespace px4_ros2
 