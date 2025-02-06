/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

using namespace std::chrono_literals; // NOLINT

static const std::string kNameFirst = "Auto-Executor-Start";
static const std::string kNameSecond = "Auto-Executor-Segment";
static const std::string kNameThird = "Auto-Executor-End";

class FlightModeTestStart : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTestStart(rclcpp::Node & node)
  : ModeBase(node, Settings{kNameFirst, false})
  {
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
  }

  ~FlightModeTestStart() override = default;

  void onActivate() override
  {
    _activation_time = node().get_clock()->now();
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    const rclcpp::Time now = node().get_clock()->now();

    if (now - _activation_time > 5s) {
      completed(px4_ros2::Result::Success);
      return;
    }

    const float elapsed_s = (now - _activation_time).seconds();
    const Eigen::Vector3f velocity{10.f, elapsed_s * 2.f, -2.f};
    _trajectory_setpoint->update(velocity);
  }

private:
  rclcpp::Time _activation_time{};
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
};

class FlightModeTestSegment : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTestSegment(rclcpp::Node & node)
  : ModeBase(node, Settings{kNameSecond, false})
  {
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
  }

  ~FlightModeTestSegment() override = default;

  void onActivate() override
  {
    _activation_time = node().get_clock()->now();
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    const rclcpp::Time now = node().get_clock()->now();

    if (now - _activation_time > 5s) {
      completed(px4_ros2::Result::Success);
      return;
    }

    const float elapsed_s = (now - _activation_time).seconds();
    const Eigen::Vector3f velocity{10.f, -elapsed_s * 2.f, 2.f};
    _trajectory_setpoint->update(velocity);
  }

private:
  rclcpp::Time _activation_time{};
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
};

class FlightModeTestEnd : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTestEnd(rclcpp::Node & node)
  : ModeBase(node, Settings{kNameThird, false})
  {
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
  }

  ~FlightModeTestEnd() override = default;

  void onActivate() override
  {
    _activation_time = node().get_clock()->now();
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    const rclcpp::Time now = node().get_clock()->now();

    if (now - _activation_time > 5s) {
      completed(px4_ros2::Result::Success);
      return;
    }

    const Eigen::Vector3f velocity{-10.f, 0, 0};
    _trajectory_setpoint->update(velocity);
  }

private:
  rclcpp::Time _activation_time{};
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
};

class ModeExecutorTest : public px4_ros2::ModeExecutorBase
{
public:
  ModeExecutorTest(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode, px4_ros2::ModeBase & second_mode, px4_ros2::ModeBase & third_mode)
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
