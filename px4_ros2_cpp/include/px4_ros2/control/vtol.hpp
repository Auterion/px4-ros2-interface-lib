/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

 #pragma once

 #include <px4_msgs/msg/vehicle_command.hpp>
 #include <px4_msgs/msg/vtol_vehicle_status.hpp>
 #include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_ros2/components/shared_subscription.hpp>
 #include <Eigen/Core>

 #include <px4_ros2/common/context.hpp>

namespace px4_ros2
{
struct VTOLConfig
{
  float back_transition_deceleration{2.f}; /**< vehicle deceleration during back-transition [m/s^2]. */
  float back_transition_deceleration_setpoint_to_pitch_i{0.1f}; /**< Backtransition deceleration setpoint to pitch I gain  [rad s/m]. */
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
    const float back_transition_deceleration_setpoint_to_pitch_i)
  {
    this->back_transition_deceleration_setpoint_to_pitch_i =
      back_transition_deceleration_setpoint_to_pitch_i;
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
    Undefined = 0,
    TransitionToFixedWing = 1,
    TransitionToMulticopter = 2,
    Multicopter = 3,
    FixedWing  = 4
  };

  /**
   * VTOL Transition
   */

  bool toMulticopter();
  bool toFixedwing();

  Eigen::Vector3f computeAccelerationSetpointDuringTransition(
    std::optional<float> back_transition_deceleration_m_s2 = std::nullopt);

  State getCurrentState() const {return _current_state;}

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
  rclcpp::Subscription<px4_msgs::msg::VtolVehicleStatus>::SharedPtr _vtol_vehicle_status_sub;
  SharedSubscriptionCallbackInstance _vehicle_local_position_cb;
  rclcpp::Time _last_command_sent;
  rclcpp::Time _last_vtol_vehicle_status_received;
  rclcpp::Time _last_pitch_integrator_update{0, 0, _node.get_clock()->get_clock_type()};

  float _vehicle_heading{NAN};
  float _decel_error_bt_int{0.f};

  Eigen::Vector2f _vehicle_acceleration_xy{NAN, NAN};

  State _current_state{State::Undefined};

  float computePitchSetpointDuringBacktransition(
    std::optional<float> back_transition_deceleration_m_s2 = std::nullopt);

  VTOLConfig _config;
};

/** @}*/
}  /* namespace px4_ros2 */
