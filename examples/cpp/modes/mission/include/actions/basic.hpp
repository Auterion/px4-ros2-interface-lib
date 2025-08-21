/****************************************************************************
* Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>

class BasicCustomAction : public px4_ros2::ActionInterface
{
public:
  explicit BasicCustomAction(px4_ros2::ModeBase & mode)
  : _node(mode.node())
  {
  }
  std::string name() const override {return "basicCustomAction";}

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    RCLCPP_INFO(_node.get_logger(), "Running custom action");
    if (arguments.contains("customArgument")) {
      RCLCPP_INFO(
        _node.get_logger(), "Custom argument: %s",
        arguments.at<std::string>("customArgument").c_str());
    }
    // Directly continue
    on_completed();
  }

private:
  rclcpp::Node & _node;
};
