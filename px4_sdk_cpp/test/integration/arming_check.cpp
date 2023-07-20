/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>

#include "util.hpp"

#include <px4_sdk/components/health_and_arming_checks.hpp>
#include <px4_sdk/components/wait_for_fmu.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class Tester : public ::testing::Test
{
public:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
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
    WaitForFirstCallback,
    WaitUntilCanArm,
    WaitUntilCannotArm,
    Completed,
  };

  void setState(State state)
  {
    RCLCPP_DEBUG(_node.get_logger(), "Executing state %i", (int)state);
    _state = state;
  }

  rclcpp::Node & _node;
  State _state{State::WaitForFirstCallback};
  rclcpp::TimerBase::SharedPtr _test_timeout;
  VehicleState _vehicle_state;
  int _num_failures_reported{0};
  std::unique_ptr<px4_sdk::HealthAndArmingChecks> _health_and_arming_checks;
};

void TestExecution::run()
{
  _test_timeout = _node.create_wall_timer(
    20s, [this] {
      EXPECT_EQ(_state, State::Completed);
      rclcpp::shutdown();
    });

  // The test does the following:
  // - wait until arming callback called
  // - wait until arming is possible
  // - report a failing check
  // - expect arming denied
  _vehicle_state.setOnVehicleStatusUpdate(
    [this](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) {
      switch (_state) {
        case State::WaitUntilCanArm:
          if (msg->pre_flight_checks_pass) {
            setState(State::WaitUntilCannotArm);
          }

          break;

        case State::WaitUntilCannotArm:

          // Ensure we got a vehicle status update after we reported a failing check (prevent race conditions)
          if (_num_failures_reported == 2) {
            ASSERT_FALSE(msg->pre_flight_checks_pass);
            setState(State::Completed);
            rclcpp::shutdown();
          }

          break;

        default:
          break;
      }
    });

  _health_and_arming_checks = std::make_unique<px4_sdk::HealthAndArmingChecks>(
    _node, [this](px4_sdk::HealthAndArmingCheckReporter &
    reporter) {
      if (_state == State::WaitForFirstCallback) {
        setState(State::WaitUntilCanArm);

      } else if (_state == State::WaitUntilCannotArm) {
        /* EVENT
                 */
        reporter.armingCheckFailureExt(
          events::ID("check_unit_test_failure"),
          events::Log::Error, "Test Failure");
        ++_num_failures_reported;
      }
    });
  ASSERT_TRUE(_health_and_arming_checks->doRegister("arming check test"));
}


TEST_F(Tester, denyArming)
{
  auto test_node = initNode();
  ASSERT_TRUE(px4_sdk::waitForFMU(*test_node, 10s));
  TestExecution test_execution{*test_node};
  test_execution.run();
  rclcpp::spin(test_node);
}
