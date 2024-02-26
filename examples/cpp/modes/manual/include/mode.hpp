/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_ros2/control/peripheral_actuators.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals; // NOLINT

static const std::string kName = "My Manual Mode";

class FlightModeTest : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, kName)
  {
    _manual_control_input = std::make_shared<px4_ros2::ManualControlInput>(*this);
    _rates_setpoint = std::make_shared<px4_ros2::RatesSetpointType>(*this);
    _attitude_setpoint = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);
    _peripheral_actuator_controls = std::make_shared<px4_ros2::PeripheralActuatorControls>(*this);
  }

  void onActivate() override {}

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    const float threshold = 0.9f;
    const bool want_rates = fabsf(_manual_control_input->roll()) > threshold || fabsf(
      _manual_control_input->pitch()) > threshold;

    const float yaw_rate = px4_ros2::degToRad(_manual_control_input->yaw() * 120.f);

    if (want_rates) {
      const Eigen::Vector3f thrust_sp{0.f, 0.f, -_manual_control_input->throttle()};
      const Eigen::Vector3f rates_sp{
        px4_ros2::degToRad(_manual_control_input->roll() * 500.f),
        px4_ros2::degToRad(-_manual_control_input->pitch() * 500.f),
        yaw_rate
      };
      _rates_setpoint->update(rates_sp, thrust_sp);

    } else {
      _yaw += yaw_rate * dt_s;
      const Eigen::Vector3f thrust_sp{0.f, 0.f, -_manual_control_input->throttle()};
      const Eigen::Quaternionf qd = px4_ros2::eulerRpyToQuaternion(
        px4_ros2::degToRad(_manual_control_input->roll() * 55.f),
        px4_ros2::degToRad(-_manual_control_input->pitch() * 55.f),
        _yaw
      );
      _attitude_setpoint->update(qd, thrust_sp, yaw_rate);
    }

    // Example to control a servo by passing through RC aux1 channel to 'Peripheral Actuator Set 1'
    _peripheral_actuator_controls->set(_manual_control_input->aux1());
  }

private:
  std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input;
  std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;
  std::shared_ptr<px4_ros2::AttitudeSetpointType> _attitude_setpoint;
  std::shared_ptr<px4_ros2::PeripheralActuatorControls> _peripheral_actuator_controls;
  float _yaw{0.f};
};
