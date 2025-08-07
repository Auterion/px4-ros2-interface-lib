/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>

static const std::string kName = "Mission Example";

class CustomAction : public px4_ros2::ActionInterface
{
public:
  explicit CustomAction(px4_ros2::ModeBase & mode)
  : _node(mode.node())
  {
  }
  std::string name() const override {return "customAction";}

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

class CustomTrajectoryAction : public px4_ros2::ActionInterface
{
public:
  explicit CustomTrajectoryAction(px4_ros2::ModeBase & mode) {}
  std::string name() const override {return "customTrajectory";}

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    // Run a custom trajectory
    auto mission = std::make_shared<px4_ros2::Mission>(
      std::vector<px4_ros2::MissionItem>{px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
        px4_ros2::Waypoint({47.39820919, 8.54595214, 510})});
    handler->runTrajectory(mission, on_completed);
  }
};

class Mission
{
public:
  explicit Mission(const std::shared_ptr<rclcpp::Node> & node)
  : _node(node)
  {
    const auto mission =
      px4_ros2::Mission(
      nlohmann::json::parse(
        R"(
  {
    "version": 1,
    "mission": {
        "defaults": {
            "horizontalVelocity": 5,
            "maxHeadingRate": 60
         },
        "items": [
           {
               "type": "takeoff"
           },
           {
               "type": "navigation",
               "navigationType": "waypoint",
               "x": 47.3977419,
               "y": 8.5455939,
               "z": 500,
               "frame": "global"
           },
           {
               "type": "navigation",
               "navigationType": "waypoint",
               "x": 47.39791657,
               "y": 8.54595214,
               "z": 500,
               "frame": "global"
           },
           {
               "type": "customAction",
               "customArgument": "custom value"
           },
           {
               "type": "customTrajectory"
           },
           {
               "type": "navigation",
               "navigationType": "waypoint",
               "x": 47.39820919,
               "y": 8.5457699,
               "z": 500,
               "frame": "global"
           },
           {
               "type": "navigation",
               "navigationType": "waypoint",
               "x": 47.39823579,
               "y": 8.54539222,
               "z": 500,
               "frame": "global"
           },
           {
               "type": "navigation",
               "navigationType": "waypoint",
               "x": 47.39797367,
               "y": 8.54497451,
               "z": 500,
               "frame": "global"
           },
           {
               "type": "rtl"
           }
        ]
    }
  }
)"));
    _mission_executor = std::make_unique<px4_ros2::MissionExecutor>(
      kName,
      px4_ros2::MissionExecutor::Configuration().addCustomAction<CustomAction>()
      .addCustomAction<CustomTrajectoryAction>(),
      *node);

    if (!_mission_executor->doRegister()) {
      throw std::runtime_error("Failed to register mission executor");
    }
    _mission_executor->setMission(mission);

    _mission_executor->onProgressUpdate(
      [&](int current_index) {
        RCLCPP_INFO(_node->get_logger(), "Current mission index: %i", current_index);
      });
    _mission_executor->onCompleted(
      [&]
      {
        RCLCPP_INFO(_node->get_logger(), "Mission completed callback");
      });

  }

private:
  std::shared_ptr<rclcpp::Node> _node;
  std::unique_ptr<px4_ros2::MissionExecutor> _mission_executor;
};
