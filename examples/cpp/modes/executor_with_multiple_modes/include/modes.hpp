/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
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
    _rates_setpoint = std::make_shared<px4_ros2::RatesSetpointType>(*this);
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

    if (now - _activation_time > 500ms) {
      completed(px4_ros2::Result::Success);
      return;
    }

    const Eigen::Vector3f rate{10.f, 0.f, 0.f};
    const Eigen::Vector3f thrust{0.f, 0.f, -0.5f};
    _rates_setpoint->update(rate, thrust);
  }

private:
  rclcpp::Time _activation_time{};
  std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;
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

    if (now - _activation_time > 10s) {
      completed(px4_ros2::Result::Success);
      return;
    }

    const unsigned int elapsed_s = static_cast<unsigned int>((now - _activation_time).seconds());
    const Eigen::Vector3f velocity{-10.f, (elapsed_s % 3 - 1) * 2.f, 0.f};
    _trajectory_setpoint->update(velocity);
  }

private:
  rclcpp::Time _activation_time{};
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
};
