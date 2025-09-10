/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cmath>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/control/setpoint_types/fixedwing/lateral_longitudinal.hpp>
#include <px4_ros2/control/vtol.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals; // NOLINT
using namespace px4_ros2::literals; // NOLINT

static const std::string kName = "My VTOL Mode";

class FlightModeTest : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, kName)
  {
    _vtol = std::make_shared<px4_ros2::VTOL>(*this);
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _fixed_wing_setpoint = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);

    _vehicle_global_position = std::make_shared<px4_ros2::OdometryGlobalPosition>(*this);
  }

  void onActivate() override
  {
    const px4_ros2::VTOL::State initial_vtol_state = _vtol->getCurrentState();

    if (initial_vtol_state == px4_ros2::VTOL::State::FixedWing) {
      _control_state = State::GoingNorth;
      RCLCPP_INFO(
        node().get_logger(),
        "VTOL Mode Activated. Flying North, climbing to 600m above sea level.");
    } else if (initial_vtol_state == px4_ros2::VTOL::State::Undefined) {
      throw std::runtime_error("VTOL state undefined.");
    } else {
      _control_state = State::TransitionToFixedWing;
      RCLCPP_INFO(node().get_logger(), "VTOL Mode Activated. Transitioning to Fixed-Wing");
    }
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    switch (_control_state) {

      case State::TransitionToFixedWing: {

          _vtol->toFixedwing();

          if (_vtol->getCurrentState() == px4_ros2::VTOL::State::TransitionToFixedWing) {

            Eigen::Vector3f acceleration_sp =
              _vtol->computeAccelerationSetpointDuringTransition();
            Eigen::Vector3f velocity_sp{NAN, NAN, 0.f};
            float course_sp = 0.f; // align vehicle north
            float height_rate_sp = 0.f;

            _trajectory_setpoint->update(velocity_sp, acceleration_sp);
            _fixed_wing_setpoint->updateWithHeightRate(height_rate_sp, course_sp);

          } else if (_vtol->getCurrentState() == px4_ros2::VTOL::State::FixedWing) {

            float altitude_sp = 600.f;
            float course_sp = 0.f; // due north

            _fixed_wing_setpoint->updateWithAltitude(altitude_sp, course_sp);

            _control_state = State::GoingNorth;
            RCLCPP_INFO(
              node().get_logger(),
              "Transition complete. Flying North, climbing to 600m above sea level.");
          }
        }
        break;

      case State::GoingNorth: {

          float altitude_sp = 600.f;
          float course_sp = 0.f; // due north

          _fixed_wing_setpoint->updateWithAltitude(altitude_sp, course_sp);

          if (_vehicle_global_position->position().z() >= (altitude_sp - 5.f)) {
            _control_state = State::TransitionToMulticopterAndHold;
            RCLCPP_INFO(
              node().get_logger(), "Altitude Reached. Transitioning to Multicopter and holding.");
          }
        }
        break;

      case State::TransitionToMulticopterAndHold: {

          _vtol->toMulticopter();

          if (_vtol->getCurrentState() == px4_ros2::VTOL::State::TransitionToMulticopter) {

            Eigen::Vector3f acceleration_sp =
              _vtol->computeAccelerationSetpointDuringTransition();
            Eigen::Vector3f velocity_sp{NAN, NAN, 0.f};
            float course_sp = 180.0_deg; // start aligning vehicle south during transition
            float height_rate_sp = 0.f;

            _trajectory_setpoint->update(velocity_sp, acceleration_sp);
            _fixed_wing_setpoint->updateWithHeightRate(height_rate_sp, course_sp);

          } else if (_vtol->getCurrentState() == px4_ros2::VTOL::State::Multicopter) {

            Eigen::Vector3f velocity_sp{0.f, 0.f, 0.f};
            Eigen::Vector3f acceleration_sp{NAN, NAN, NAN};
            float heading_sp = 180.0_deg;

            _trajectory_setpoint->update(velocity_sp, acceleration_sp, heading_sp);
          }
        }
    }
  }

private:
  std::shared_ptr<px4_ros2::VTOL> _vtol;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  std::shared_ptr<px4_ros2::FwLateralLongitudinalSetpointType> _fixed_wing_setpoint;
  std::shared_ptr<px4_ros2::OdometryGlobalPosition> _vehicle_global_position;


  enum class State
  {
    TransitionToFixedWing = 0,
    GoingNorth,
    TransitionToMulticopterAndHold
  } _control_state;
};
