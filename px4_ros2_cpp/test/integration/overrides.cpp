/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>

#include "util.hpp"

#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

using namespace std::chrono_literals;

static const std::string kName = "Test Flight Mode";

namespace override_tests
{

class FlightModeTest : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, Settings{kName, false})
  {
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
  }

  ~FlightModeTest() override = default;

  void onActivate() override
  {
    _activation_time = node().get_clock()->now();
    ++num_activations;
  }

  void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter & reporter) override
  {
    ++num_arming_check_updates;
  }

  void onDeactivate() override
  {
    ++num_deactivations;
  }

  void updateSetpoint(float dt_s) override
  {
    ++num_setpoint_updates;
    const rclcpp::Time now = node().get_clock()->now();

    if (now - _activation_time > 8s) {
      completed(px4_ros2::Result::Success);
      return;
    }

    // Send some random setpoints
    const float elapsed_s = (now - _activation_time).seconds();
    const Eigen::Vector3f velocity{5.f, elapsed_s, 0.f};
    _trajectory_setpoint->update(velocity);
  }

  int num_activations{0};
  int num_deactivations{0};
  int num_setpoint_updates{0};
  int num_arming_check_updates{0};

private:
  rclcpp::Time _activation_time{};
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
};

class ModeExecutorTest : public px4_ros2::ModeExecutorBase
{
public:
  ModeExecutorTest(rclcpp::Node & node, FlightModeTest & owned_mode, bool activate_immediately)
  : ModeExecutorBase(node,
      ModeExecutorBase::Settings{activate_immediately ? Settings::Activation::ActivateImmediately :
        Settings::Activation::ActivateOnlyWhenArmed},
      owned_mode),
    _node(node)
  {}

  enum class State
  {
    Reset = 0,
    WaitForArming = 1,
    Arming = 2,
    TakingOff = 3,
    Landing = 4,
    Wait = 5,
    TakingOff2 = 6,
    WaitForFailsafe = 7,
    WaitForDisarming = 8,
  };

  void onActivate() override
  {
    ++num_activations;
    runState(State::WaitForArming, px4_ros2::Result::Success);
  }

  void onDeactivate(DeactivateReason reason) override
  {
    EXPECT_EQ(_state, State::WaitForDisarming);
    ++num_deactivations;
  }

  void onFailsafeDeferred() override
  {
    ++num_failsafe_deferred;
    EXPECT_EQ(_state, State::WaitForFailsafe);
    // Re-enable failsafes -> failsafe will trigger and executor deactivated
    deferFailsafesSync(false);
    runState(State::WaitForDisarming, px4_ros2::Result::Success);
  }

  void runState(State state, px4_ros2::Result previous_result)
  {
    on_state_completed(state, previous_result);

    if (previous_result != px4_ros2::Result::Success) {
      RCLCPP_ERROR(
        _node.get_logger(), "State %i: previous state failed: %s", (int)state,
        resultToString(previous_result));
      return;
    }

    RCLCPP_DEBUG(_node.get_logger(), "Executing state %i", (int)state);

    _state = state;

    switch (state) {
      case State::Reset:
        break;

      case State::WaitForArming:
        waitReadyToArm([this](px4_ros2::Result result) {runState(State::Arming, result);});
        break;

      case State::Arming:
        arm([this](px4_ros2::Result result) {runState(State::TakingOff, result);});
        break;

      case State::TakingOff:
        takeoff([this](px4_ros2::Result result) {runState(State::Landing, result);});
        break;

      case State::Landing:
        land([this](px4_ros2::Result result) {runState(State::Wait, result);});
        break;

      case State::Wait:
        _wait_timer = _node.create_wall_timer(
          10s, [this] {
            _wait_timer.reset();
            EXPECT_TRUE(isArmed());
            runState(State::TakingOff2, px4_ros2::Result::Success);
          });
        break;

      case State::TakingOff2:
        takeoff(
          [this](px4_ros2::Result result) {
            EXPECT_TRUE(deferFailsafesSync(true, 60));
            runState(State::WaitForFailsafe, result);
          });
        break;

      case State::WaitForFailsafe:
      case State::WaitForDisarming:
        // Nothing to do
        break;

    }
  }

  State getState() const {return _state;}

  int num_activations{0};
  int num_deactivations{0};
  int num_failsafe_deferred{0};

  std::function<void()> on_completed;
  std::function<void(State, px4_ros2::Result)> on_state_completed;

private:
  State _state{};
  rclcpp::Node & _node;
  rclcpp::TimerBase::SharedPtr _wait_timer;
};


class TestExecutionOverrides
{
public:
  explicit TestExecutionOverrides(rclcpp::Node & node)
  : _node(node), _vehicle_state(node) {}

  void run();

private:
  rclcpp::Node & _node;
  rclcpp::TimerBase::SharedPtr _test_timeout;

  std::unique_ptr<FlightModeTest> _mode;
  std::unique_ptr<ModeExecutorTest> _mode_executor;
  VehicleState _vehicle_state;
  bool _was_armed{false};
};

void TestExecutionOverrides::run()
{
  _test_timeout = _node.create_wall_timer(
    200s, [] {
      EXPECT_TRUE(false) << "Timeout";
      rclcpp::shutdown();
    });

  _mode = std::make_unique<FlightModeTest>(_node);
  _mode_executor = std::make_unique<ModeExecutorTest>(_node, *_mode, true);


  // Testing steps:
  // - disable auto-disarm
  // - register & activate executor
  // - takeoff
  // - land & ensure no auto-disarm triggered
  // - takeoff
  // - defer failsafes & trigger failsafe
  // - wait for callback & enable failsafes
  // - wait for failsafe triggering, landing and disarming

  _mode_executor->on_state_completed =
    [this](ModeExecutorTest::State next_state, px4_ros2::Result result) {
      if (next_state == ModeExecutorTest::State::WaitForFailsafe) {
        EXPECT_EQ(result, px4_ros2::Result::Success);
        // trigger failsafe (one that can be deferred)
        _vehicle_state.setForceLowBattery(true);

      } else {
        EXPECT_EQ(result, px4_ros2::Result::Success);
      }
    };

  _vehicle_state.setOnVehicleStatusUpdate(
    [this](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) {
      const bool armed = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

      if (_was_armed && !armed) {
        // Finish the test
        _vehicle_state.setForceLowBattery(false);

        EXPECT_EQ(_mode_executor->getState(), ModeExecutorTest::State::WaitForDisarming);
        EXPECT_EQ(_mode_executor->num_activations, 1);
        EXPECT_EQ(_mode_executor->num_deactivations, 1);
        EXPECT_EQ(_mode_executor->num_failsafe_deferred, 1);

        rclcpp::shutdown();
      }

      _was_armed = armed;
    });

  _mode_executor->configOverrides().controlAutoDisarm(false);
  ASSERT_TRUE(_mode_executor->doRegister());
}

TEST_F(ModesTest, runExecutorOverrides)
{
  auto test_node = initNode();
  ASSERT_TRUE(px4_ros2::waitForFMU(*test_node, 10s));
  TestExecutionOverrides test_execution{*test_node};
  test_execution.run();
  rclcpp::spin(test_node);
}

} // namespace override_tests
