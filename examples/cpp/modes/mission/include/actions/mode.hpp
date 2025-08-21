/****************************************************************************
* Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>

class FlightModeTest : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, Settings{"My Mode"})
  {
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
  }

  void onActivate() override
  {
    _duration = 0.0f;
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    _duration += dt_s;
    if (_duration > 5.0f) {
      completed(px4_ros2::Result::Success);
    }
    // Send a fixed velocity setpoint
    _trajectory_setpoint->update(Eigen::Vector3f{2.0f, 3.0f, 0.0f});
  }

private:
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  float _duration{0.0f};
};

class ModeAction : public px4_ros2::ActionInterface
{
public:
  explicit ModeAction(px4_ros2::ModeBase & mode)
  : _node(mode.node())
  {
    _mode = std::make_shared<FlightModeTest>(_node);

    if (!_mode->doRegister()) {
      throw std::runtime_error("Registration failed");
    }
  }
  std::string name() const override {return "myMode";}

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    RCLCPP_INFO(_node.get_logger(), "Running custom mode");
    handler->runMode(_mode->id(), on_completed);
  }

private:
  rclcpp::Node & _node;
  std::shared_ptr<px4_ros2::ModeBase> _mode;
};
