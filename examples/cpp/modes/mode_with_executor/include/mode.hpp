/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_sdk/components/mode.hpp>
#include <px4_sdk/components/mode_executor.hpp>
#include <px4_sdk/components/wait_for_fmu.hpp>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

using namespace std::chrono_literals; // NOLINT

static const char * name = "Autonomous Executor";
static const char * node_name = "example_mode_with_executor";

class FlightModeTest : public px4_sdk::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, Settings{name, false}, px4_sdk::ModeRequirements::autonomous())
  {
    setSetpointUpdateRate(30.F);
  }

  ~FlightModeTest() override = default;

  void onActivate() override
  {
    _activation_time = node().get_clock()->now();
    setpoints().configureSetpointsSync(px4_sdk::SetpointSender::SetpointConfiguration{});
  }

  void onDeactivate() override {}

  void updateSetpoint() override
  {
    rclcpp::Time now = node().get_clock()->now();

    if (now - _activation_time > rclcpp::Duration::from_seconds(5)) {
      completed(px4_sdk::Result::Success);
      return;
    }

    const float elapsed_s = (now - _activation_time).seconds();
    const Eigen::Vector3f velocity{10.F, elapsed_s * 2.F, -2.F};
    setpoints().sendTrajectorySetpoint(velocity);
  }

private:
  rclcpp::Time _activation_time{};
};

class ModeExecutorTest : public px4_sdk::ModeExecutorBase
{
public:
  ModeExecutorTest(rclcpp::Node & node, px4_sdk::ModeBase & owned_mode)
  : ModeExecutorBase(node, px4_sdk::ModeExecutorBase::Settings{}, owned_mode),
    _node(node)
  {
  }

  enum class State
  {
    Reset,
    TakingOff,
    MyMode,
    RTL,
    WaitUntilDisarmed,
  };

  void onActivate() override
  {
    runState(State::TakingOff, px4_sdk::Result::Success);
  }

  void onDeactivate(DeactivateReason reason) override
  {
  }

  void runState(State state, px4_sdk::Result previous_result)
  {
    if (previous_result != px4_sdk::Result::Success) {
      RCLCPP_ERROR(
        _node.get_logger(), "State %i: previous state failed: %s", (int)state,
        resultToString(previous_result));
      return;
    }

    RCLCPP_DEBUG(_node.get_logger(), "Executing state %i", (int)state);

    switch (state) {
      case State::Reset:
        break;

      case State::TakingOff:
        takeoff([this](px4_sdk::Result result) {runState(State::MyMode, result);});
        break;

      case State::MyMode:
        scheduleMode(
          ownedMode().id(), [this](px4_sdk::Result result) {
            runState(State::RTL, result);
          });
        break;

      case State::RTL:
        rtl([this](px4_sdk::Result result) {runState(State::WaitUntilDisarmed, result);});
        break;

      case State::WaitUntilDisarmed:
        waitUntilDisarmed(
          [this](px4_sdk::Result result) {
            RCLCPP_INFO(_node.get_logger(), "All states complete (%s)", resultToString(result));
          });
        break;
    }
  }

private:
  rclcpp::Node & _node;
};

class TestNode : public rclcpp::Node
{
public:
  TestNode()
  : Node(node_name)
  {
    // Enable debug output
    auto ret =
      rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
    }

    if (!px4_sdk::waitForFMU(*this)) {
      throw std::runtime_error("No message from FMU");
    }

    _mode = std::make_unique<FlightModeTest>(*this);
    _mode_executor = std::make_unique<ModeExecutorTest>(*this, *_mode);

    if (!_mode_executor->doRegister()) {
      throw std::runtime_error("Registration failed");
    }
  }

private:
  std::unique_ptr<FlightModeTest> _mode;
  std::unique_ptr<ModeExecutorTest> _mode_executor;
};
