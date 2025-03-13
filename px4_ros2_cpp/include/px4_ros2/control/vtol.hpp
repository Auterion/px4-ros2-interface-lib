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
  * @brief Provides VTOL transition command
  *
  */
 class VTOL
 {
 public:
 
   explicit VTOL(Context & context);
 
   /**
    * VTOL Transition from FW to MC
    */
   void transitionFWtoMC();
 
   void transitionMCtoFW(); 
   
 
   private:
   rclcpp::Node & _node;
   rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
   // rclcpp::Time _last_update{};
 };
 
 /** @}*/
 } /* namespace px4_ros2 */
 