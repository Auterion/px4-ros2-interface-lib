/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

 #include <px4_ros2/control/vtol.hpp>
 #include <px4_ros2/utils/message_version.hpp>
 #include <rclcpp/rclcpp.hpp>


using namespace std::chrono_literals;

static constexpr float kConstantsOneG = 9.80665f; // m/s^2

namespace px4_ros2
{

VTOL::VTOL(Context & context, const VTOLConfig & config)
: _node(context.node()), _config(config)
{
  _vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>(
    context.topicNamespacePrefix() + "fmu/in/vehicle_command" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommand>(),
    1);

  _vtol_vehicle_status_sub = _node.create_subscription<px4_msgs::msg::VtolVehicleStatus>(
    context.topicNamespacePrefix() + "fmu/out/vtol_vehicle_status" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VtolVehicleStatus>(),
    rclcpp::QoS(10).best_effort(),
    [this](px4_msgs::msg::VtolVehicleStatus::UniquePtr msg) {

      _last_vtol_vehicle_status_received = _node.get_clock()->now();

      switch (msg->vehicle_vtol_state) {
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

  _vehicle_local_position_sub = _node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    context.topicNamespacePrefix() + "fmu/out/vehicle_local_position" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>(),
    rclcpp::QoS(10).best_effort(),
    [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
      _vehicle_heading = msg->heading;
      _vehicle_acceleration_xy = {msg->ax, msg->ay};

    });

  _last_command_sent = _node.get_clock()->now();

}

VTOL::State VTOL::getCurrentState()
{
  return _current_state;
}

bool VTOL::toMulticopter()
{
  const auto now = _node.get_clock()->now();

  if (now - _last_vtol_vehicle_status_received < 2s) {
    if ((_current_state == VTOL::State::FIXED_WING ||
      _current_state == VTOL::State::TRANSITION_TO_FIXED_WING) &&
      (now - _last_command_sent) > 150ms)
    {
      _last_command_sent = now;

      px4_msgs::msg::VehicleCommand cmd;

      cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION;
      cmd.param1 = px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_MC;

      cmd.param2 = 0;

      cmd.target_system = 0;
      cmd.target_component = 1;

      _vehicle_command_pub->publish(cmd);
    }
    return true;
  }

  RCLCPP_WARN(_node.get_logger(), "Current VTOL vehicle state unknown. Not able to transition.");
  return false;
}

bool VTOL::toFixedwing()
{
  const auto now = _node.get_clock()->now();

  if (now - _last_vtol_vehicle_status_received < 2s) {
    if ((_current_state == VTOL::State::MULTICOPTER ||
      _current_state == VTOL::State::TRANSITION_TO_MULTICOPTER) &&
      (now - _last_command_sent) > 150ms)
    {
      _last_command_sent = now;

      px4_msgs::msg::VehicleCommand cmd;

      cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION;
      cmd.param1 = px4_msgs::msg::VtolVehicleStatus::VEHICLE_VTOL_STATE_FW;

      cmd.param2 = 0;

      cmd.target_system = 0;
      cmd.target_component = 1;

      _vehicle_command_pub->publish(cmd);
    }
    return true;
  }

  RCLCPP_WARN(_node.get_logger(), "Current VTOL vehicle state unknown. Not able to transition.");
  return false;
}

Eigen::Vector3f VTOL::computeAccelerationSetpointDuringTransition(
  std::optional<float> back_transition_deceleration_m_s2)
{
  const Eigen::Vector2f velocity_xy_direction = {std::cos(_vehicle_heading), std::sin(
      _vehicle_heading)};
  const bool is_backtransition = VTOL::getCurrentState() ==
    VTOL::State::TRANSITION_TO_MULTICOPTER;

  float pitch_setpoint = 0.f;

  if (is_backtransition) {
    pitch_setpoint =
      computePitchSetpointDuringBacktransition(back_transition_deceleration_m_s2);
  }

  Eigen::Vector2f acceleration_setpoint_during_transition = tanf(pitch_setpoint) * kConstantsOneG *
    -velocity_xy_direction;

  return {acceleration_setpoint_during_transition.x(), acceleration_setpoint_during_transition.y(),
    NAN};
}

float VTOL::computePitchSetpointDuringBacktransition(
  std::optional<float> back_transition_deceleration_m_s2)
{

  const float deceleration_setpoint = back_transition_deceleration_m_s2.value_or(
    _config.back_transition_deceleration);
  const Eigen::Vector2f velocity_xy_direction = {std::cos(_vehicle_heading), std::sin(
      _vehicle_heading)};

  // Pitch up to reach a negative accel_in_flight_direction otherwise we decelerate too slow
  const float deceleration = -_vehicle_acceleration_xy.dot(velocity_xy_direction);
  const float deceleration_error = deceleration_setpoint - deceleration;

  const auto now = _node.get_clock()->now();
  float dt = (now - _last_pitch_integrator_update).seconds();
  _last_pitch_integrator_update = now;

  // Reset for new transition
  if (dt > 2.f) {
    dt = 0.f;
    _decel_error_bt_int = 0.f;
  }

  // Update back-transition deceleration error integrator
  _decel_error_bt_int +=
    (_config.back_transition_deceleration_setpoint_to_pitch_I * deceleration_error) * dt;
  _decel_error_bt_int = std::clamp(_decel_error_bt_int, 0.f, _config.deceleration_integrator_limit);

  return _decel_error_bt_int;
}

}// namespace px4_ros2
