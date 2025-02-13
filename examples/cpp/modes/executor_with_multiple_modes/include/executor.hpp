/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <modes.hpp>
#include <px4_ros2/components/mode_executor.hpp>

class ModeExecutorTest : public px4_ros2::ModeExecutorBase
{
public:
  ModeExecutorTest(
    rclcpp::Node & node, px4_ros2::ModeBase & owned_mode,
    px4_ros2::ModeBase & second_mode, px4_ros2::ModeBase & third_mode)
  : ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{}, owned_mode),
    _node(node), _second_mode(second_mode), _third_mode(third_mode)
  {
  }

  enum class State
  {
    Reset,
    TakingOff,
    MyFirstMode,
    MySecondMode,
    MyThirdMode,
    RTL,
    WaitUntilDisarmed,
  };

  void onActivate() override
  {
    runState(State::TakingOff, px4_ros2::Result::Success);
  }

  void onDeactivate(DeactivateReason reason) override
  {
  }

  void runState(State state, px4_ros2::Result previous_result)
  {
    if (previous_result != px4_ros2::Result::Success) {
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
        takeoff([this](px4_ros2::Result result) {runState(State::MyFirstMode, result);});
        break;

      case State::MyFirstMode:
        scheduleMode(
          ownedMode().id(), [this](px4_ros2::Result result) {
            runState(State::MySecondMode, result);
          });
        break;

      case State::MySecondMode:
        scheduleMode(
          _second_mode.id(), [this](px4_ros2::Result result) {
            runState(State::MyThirdMode, result);
          });
        break;

      case State::MyThirdMode:
        scheduleMode(
          _third_mode.id(), [this](px4_ros2::Result result) {
            runState(State::RTL, result);
          });
        break;

      case State::RTL:
        rtl([this](px4_ros2::Result result) {runState(State::WaitUntilDisarmed, result);});
        break;

      case State::WaitUntilDisarmed:
        waitUntilDisarmed(
          [this](px4_ros2::Result result) {
            RCLCPP_INFO(_node.get_logger(), "All states complete (%s)", resultToString(result));
          });
        break;
    }
  }

private:
  rclcpp::Node & _node;
  px4_ros2::ModeBase & _second_mode;
  px4_ros2::ModeBase & _third_mode;
};
