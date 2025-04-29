/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

 #pragma once

 #include <px4_msgs/msg/vehicle_command.hpp>
 #include <px4_msgs/msg/vehicle_status.hpp>
 #include <px4_msgs/msg/vtol_vehicle_status.hpp>
 #include <px4_msgs/msg/vehicle_local_position.hpp>
 #include <Eigen/Core>

 #include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
struct VTOLConfig
{
  float back_transition_deceleration{2.f}; /**< vehicle deceleration during back-transition [m/s^2]. */

  VTOLConfig & withBackTransitionDeceleration(
    const float back_transition_deceleration)
  {
    this->back_transition_deceleration = back_transition_deceleration;
    return *this;
  }
};

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
  explicit VTOL(Context & context, const VTOLConfig & config = VTOLConfig{});

  enum class State
  {
    UNDEFINED = 0,
    TRANSITION_TO_FIXED_WING = 1,
    TRANSITION_TO_MULTICOPTER = 2,
    MULTICOPTER = 3,
    FIXED_WING  = 4
  };

  /**
   * VTOL Transition
   */

  void to_multicopter();
  void to_fixedwing();

  Eigen::Vector3f compute_acceleration_setpoint_during_transition(
    std::optional<float> back_transition_deceleration_m_s2 = std::nullopt);

  VTOL::State get_current_state();

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
  rclcpp::Subscription<px4_msgs::msg::VtolVehicleStatus>::SharedPtr _vtol_vehicle_status_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _vehicle_local_position_sub;
  rclcpp::Time _last_command_sent;
  rclcpp::Time _last_vtol_vehicle_status_received;

  px4_msgs::msg::VehicleStatus::UniquePtr _vehicle_status_msg;
  px4_msgs::msg::VtolVehicleStatus::UniquePtr _vtol_vehicle_status_msg;
  px4_msgs::msg::VehicleLocalPosition::UniquePtr _vehicle_local_position_msg;

  uint _system_id;
  uint _component_id;

  float _vehicle_heading{0.f};

  Eigen::Vector2f _vehicle_velocity_xy{NAN, NAN};

  VTOL::State _current_state{VTOL::State::UNDEFINED};

  VTOLConfig _config;

};

/** @}*/
}  /* namespace px4_ros2 */
