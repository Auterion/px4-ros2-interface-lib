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
  float back_transition_deceleration_setpoint_to_pitch_I{0.1f}; /**< Backtransition deceleration setpoint to pitch I gain  [rad s/m]. */
  float deceleration_integrator_limit{0.3f};

  VTOLConfig & withBackTransitionDeceleration(
    const float back_transition_deceleration)
  {
    this->back_transition_deceleration = back_transition_deceleration;
    return *this;
  }

  VTOLConfig & withDecelerationIntegratorLimit(
    const float deceleration_integrator_limit)
  {
    this->deceleration_integrator_limit = deceleration_integrator_limit;
    return *this;
  }

  VTOLConfig & withBackTransitionDecelerationIGain(
    const float back_transition_deceleration_setpoint_to_pitch_I)
  {
    this->back_transition_deceleration_setpoint_to_pitch_I =
      back_transition_deceleration_setpoint_to_pitch_I;
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
  rclcpp::Time _last_pitch_integrator_update{0, 0, _node.get_clock()->get_clock_type()};

  px4_msgs::msg::VehicleStatus::UniquePtr _vehicle_status_msg;
  px4_msgs::msg::VtolVehicleStatus::UniquePtr _vtol_vehicle_status_msg;
  px4_msgs::msg::VehicleLocalPosition::UniquePtr _vehicle_local_position_msg;

  uint _system_id;
  uint _component_id;

  float _vehicle_heading{NAN};
  float _decel_error_bt_int{0.f};

  Eigen::Vector2f _vehicle_velocity_xy{NAN, NAN};
  Eigen::Vector2f _vehicle_acceleration_xy{NAN, NAN};

  VTOL::State _current_state{VTOL::State::UNDEFINED};

  float compute_pitch_setpoint_during_backtransition(
    std::optional<float> back_transition_deceleration_m_s2 = std::nullopt);

  VTOLConfig _config;
};

/** @}*/
}  /* namespace px4_ros2 */
