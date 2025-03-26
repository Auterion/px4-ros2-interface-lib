/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

 #pragma once

 #include <px4_msgs/msg/vehicle_command.hpp>
 #include <px4_msgs/msg/vehicle_status.hpp>
 #include <px4_msgs/msg/vtol_vehicle_status.hpp>
 #include <Eigen/Core>
 
 #include <px4_ros2/common/setpoint_base.hpp>
 
 namespace px4_ros2
 {
 /** \ingroup control
  *  @{
  */
 /**
  * @brief Provides VTOL transition command
  *
  */
 class VTOL
 {
 public:
 
   explicit VTOL(Context & context);
 
   /**
    * VTOL Transition
    */

   void transition(); 
   
   private:
   rclcpp::Node & _node;
   rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
   rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub; 

   px4_msgs::msg::VehicleStatus::UniquePtr _vehicle_status_msg;

   uint _system_id; 
   uint _component_id; 
   uint _vehicle_type; 
   
 };
 
 /** @}*/
 } /* namespace px4_ros2 */
 