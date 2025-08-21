/****************************************************************************
* Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>

/**
 * @brief Example camera trigger action
 *
 * This is an example for a continuous action, i.e. an action that is ongoing in the background while the mission
 * continues.
 * By storing the state of the action while it is running, the state is restored when the mission is interrupted
 * and resumed later on.
 */
class CameraTriggerAction : public px4_ros2::ActionInterface
{
public:
  explicit CameraTriggerAction(px4_ros2::ModeBase & mode)
  : _node(mode.node()) {}
  std::string name() const override {return "cameraTrigger";}

  bool canRun(
    const px4_ros2::ActionArguments & arguments,
    std::vector<std::string> & errors) override
  {
    // Require an 'action' argument
    if (!arguments.contains("action")) {
      errors.emplace_back("cameraTrigger action must have 'action' argument");
      return false;
    }
    return true;
  }

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    const auto action = arguments.at<std::string>("action");
    if (action == "start") {
      _state = handler->storeState(name(), arguments);
      startCameraTrigger();
    } else {
      _state.reset();
      stopCameraTrigger();
    }

    on_completed();
  }

  void deactivate() override
  {
    if (_state != nullptr) {
      _state.reset();
      stopCameraTrigger();
    }
  }

private:
  void startCameraTrigger()
  {
    RCLCPP_INFO(_node.get_logger(), "Starting camera trigger");
  }
  void stopCameraTrigger()
  {
    RCLCPP_INFO(_node.get_logger(), "Stopping camera trigger");
  }
  std::unique_ptr<px4_ros2::ActionStateKeeper> _state;
  rclcpp::Node & _node;
};
