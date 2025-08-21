/****************************************************************************
* Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>

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
