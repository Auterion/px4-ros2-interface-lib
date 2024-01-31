/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>

#include "util.hpp"

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

static const std::string kName = "Test Descend";

namespace mode_tests
{

class FlightModeTest : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, Settings{kName, true, ModeBase::kModeIDDescend})
  {
    _attitude_setpoint = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);
  }

  ~FlightModeTest() override = default;

  void onActivate() override
  {
    ++num_activations;
  }

  void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter & reporter) override
  {
    if (check_should_fail) {
      /* EVENT
                 */
      reporter.armingCheckFailureExt(
        px4_ros2::events::ID("check_custom_mode_test_failure"),
        px4_ros2::events::Log::Error, "Custom check failed");
    }

    ++num_arming_check_updates;
  }

  void onDeactivate() override
  {
    ++num_deactivations;
  }

  void updateSetpoint(float dt_s) override
  {
    ++num_setpoint_updates;

    // Send some random setpoints, make sure it stays in the air, we don't want it to land
    const Eigen::Vector3f thrust{0.f, 0.f, -0.6f};
    const Eigen::Quaternionf attitude{1.f, 0.f, 0.f, 0.f};
    _attitude_setpoint->update(attitude, thrust);
  }

  int num_activations{0};
  int num_deactivations{0};
  int num_setpoint_updates{0};
  int num_arming_check_updates{0};
  bool check_should_fail{false};

private:
  std::shared_ptr<px4_ros2::AttitudeSetpointType> _attitude_setpoint;
};


class TestExecution
{
public:
  explicit TestExecution(rclcpp::Node & node)
  : _node(node), _vehicle_state(node) {}

  void run();

private:
  enum class State
  {
    ActivatingLand,
    WaitForExternalMode,
    WaitForArming,
    WaitUntilInAir,
    WaitForCustomMode,
    WaitForHold,
    WaitForFailsafe,
    TerminateMode,
    WaitForDisarm,
  };

  State _state{State::ActivatingLand};

  rclcpp::Node & _node;
  rclcpp::TimerBase::SharedPtr _test_timeout;
  rclcpp::TimerBase::SharedPtr _testing_timer;

  std::unique_ptr<FlightModeTest> _mode;
  VehicleState _vehicle_state;
  bool _was_armed{false};
  uint8_t _current_nav_state{};
  int _num_pre_flight_checks_pass{0};
};

void TestExecution::run()
{
  _test_timeout = _node.create_wall_timer(
    120s, [] {
      EXPECT_TRUE(false) << "Timeout";
      rclcpp::shutdown();
    });

  _mode = std::make_unique<FlightModeTest>(_node);

  // Testing steps:
  // - switch to descend
  // - register mode, replace internal descend
  // - ensure fmu switched to external mode
  // - switch to takeoff
  // - wait for arming
  // - takeoff + wait
  // - trigger extra failing mode check
  // - try to switch into custom mode -> prevented
  // - clear extra check
  // - try to switch into custom mode -> works
  // - switch to hold
  // - trigger failsafe
  // - ensure switch to external descend
  // - terminate mode -> switch to internal
  // - wait for disarming


  // First, switch to Descend and wait for it
  RCLCPP_INFO(_node.get_logger(), "Activating Land");
  _vehicle_state.sendCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE,
    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND);
  _vehicle_state.callbackOnModeSet(
    [this]() {
      _state = State::WaitForExternalMode;
      RCLCPP_INFO(_node.get_logger(), "Registering");
      ASSERT_TRUE(_mode->doRegister());

      // We expect to switch into our custom mode, as it replaces the currently selected one
      _vehicle_state.callbackOnModeSet(
        [this]() {
          _vehicle_state.sendCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE,
            px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF);
          _state = State::WaitForArming;
          RCLCPP_INFO(_node.get_logger(), "Wait for arming");

        }, _mode->id());

    }, px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND);


  _vehicle_state.setOnVehicleStatusUpdate(
    [this](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) {
      const bool armed = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
      _current_nav_state = msg->nav_state;

      if (_state == State::WaitForArming) {
        if (msg->pre_flight_checks_pass) {
          ++_num_pre_flight_checks_pass;

        } else {
          _num_pre_flight_checks_pass = 0;
        }

        if (_num_pre_flight_checks_pass >
        3)                       // Make sure we check after the mode switch happened (as we don't wait for an ack)
        {
          RCLCPP_INFO(_node.get_logger(), "Arming possible");
          _state = State::WaitUntilInAir;
          _mode->check_should_fail = true;
          _vehicle_state.sendCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.f);
          _testing_timer = _node.create_wall_timer(
            5s, [this] {
              EXPECT_TRUE(_was_armed);
              _testing_timer.reset();
              RCLCPP_INFO(_node.get_logger(), "In air, checking mode switch prevented");
              _vehicle_state.sendCommand(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE,
                px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND);

              _testing_timer = _node.create_wall_timer(
                1s, [this] {
                  _testing_timer.reset();
                  // Mode did not switch to Descend/our mode, as the custom check prevented it
                  EXPECT_NE(
                    _current_nav_state,
                    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND);
                  EXPECT_NE(_current_nav_state, _mode->id());
                  RCLCPP_INFO(_node.get_logger(), "Current mode: %i", _current_nav_state);

                  // Now clear the check and try again
                  _mode->check_should_fail = false;
                  _testing_timer = _node.create_wall_timer(
                    1s, [this] {
                      _testing_timer.reset();
                      RCLCPP_INFO(_node.get_logger(), "In air, checking mode switch again");
                      // This must activate our mode
                      _vehicle_state.sendCommand(
                        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE,
                        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND);
                      _state = State::WaitForCustomMode;
                    });
                });
            });
        }

      } else if (_state == State::WaitForCustomMode) {
        if (msg->nav_state == _mode->id()) {
          RCLCPP_INFO(_node.get_logger(), "Custom mode active, switching to hold");
          _vehicle_state.sendCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE,
            px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER);
          _state = State::WaitForHold;
        }

      } else if (_state == State::WaitForHold) {
        if (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER) {
          RCLCPP_INFO(_node.get_logger(), "Hold mode active, triggering failsafe");
          _vehicle_state.setGPSFailure(true);
          _state = State::WaitForFailsafe;

          // Failsafe must switch into our mode
          _vehicle_state.callbackOnModeSet(
            [this]() {
              RCLCPP_INFO(_node.get_logger(), "Custom mode got activated, stopping custom mode");
              _state = State::TerminateMode;

              // Now stop the mode
              EXPECT_GT(_mode->num_activations, 0);
              EXPECT_GT(_mode->num_deactivations, 0);
              EXPECT_GT(_mode->num_setpoint_updates, 1);
              EXPECT_GT(_mode->num_arming_check_updates, 1);
              _mode.reset();

              // The FMU must fall back to the internal mode
              _vehicle_state.callbackOnModeSet(
                [this]() {
                  RCLCPP_INFO(
                    _node.get_logger(),
                    "Descend mode got activated, waiting for landing & disarm");
                  _state = State::WaitForDisarm;
                }, px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND);

            }, _mode->id());
        }
      }

      if (_was_armed && !armed && _state == State::WaitForDisarm) {
        // Disarming, complete the test
        _vehicle_state.setGPSFailure(false);
        rclcpp::shutdown();
      }

      _was_armed = armed;
    });
}

TEST_F(ModesTest, runModeTests)
{
  auto test_node = initNode();
  ASSERT_TRUE(px4_ros2::waitForFMU(*test_node, 10s));
  TestExecution test_execution{*test_node};
  test_execution.run();
  rclcpp::spin(test_node);
}
} // namespace mode_tests
