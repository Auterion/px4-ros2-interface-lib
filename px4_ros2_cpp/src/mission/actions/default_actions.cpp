/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/mission/actions/default_actions.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/visit.hpp>

namespace px4_ros2::default_actions
{

void OnResume::run(
  const std::shared_ptr<ActionHandler> & handler, const ActionArguments & arguments,
  const std::function<void()> & on_completed)
{
  const auto action = arguments.at<std::string>("action");
  // if action == "storeState", we could store the current position with handler->storeState(),
  // and use it when resuming.
  if (action != "resume") {
    return;
  }

  const int current_mission_index = handler->getCurrentMissionIndex().value_or(0);
  if (current_mission_index == 0) { // NOLINT(bugprone-branch-clone)
    // Mission starts from beginning, directly continue
    on_completed();
  } else if (_landed->lastValid(3s) && _landed->landed()) {
    // Resuming from landed state
    if (handler->currentActionSupportsResumeFromLanded()) {
      RCLCPP_DEBUG(
        _node.get_logger(),
        "Resume: current action supports resuming from landed, continuing directly");
      on_completed();
    } else {
      resumeFromUnexpectedLanding(handler, on_completed, current_mission_index);
    }
  } else {
    // This happens when the mission is resumed in-air
    // Nothing to do, directly continue (here we could fly to the nearest point on the trajectory).
    on_completed();
  }
}

void OnResume::resumeFromUnexpectedLanding(
  const std::shared_ptr<ActionHandler> & handler,
  const std::function<void()> & on_completed, int current_mission_index)
{
  const auto & mission = handler->mission();

  // If the current action is rtl or land, skip it & continue
  if (mission.indexValid(current_mission_index)) {
    if (const auto * action_item =
      std::get_if<ActionItem>(&mission.items()[current_mission_index]))
    {
      if (action_item->name == "rtl" || action_item->name == "land") {
        RCLCPP_DEBUG(_node.get_logger(), "Resume: skipping action %s", action_item->name.c_str());
        // Continue at the next index (or the first one)
        if (current_mission_index + 1 >= static_cast<int>(mission.items().size())) {
          handler->setCurrentMissionIndex(0);
        } else {
          handler->setCurrentMissionIndex(current_mission_index + 1);
        }

        on_completed();
        return;
      }
    }
  }

  // Find takeoff action and run it (if there are multiple, take the one closest to the current index)
  const ActionItem * takeoff_action_item{nullptr};
  for (int i = 0; i < current_mission_index && mission.indexValid(i); ++i) {
    const auto & item = mission.items()[i];
    if (const auto * action_item = std::get_if<ActionItem>(&item)) {
      if (action_item->name == "takeoff") {
        takeoff_action_item = action_item;
      }
    }
  }
  if (takeoff_action_item) {
    handler->runAction(
      takeoff_action_item->name, takeoff_action_item->arguments,
      [this, handler, on_completed, current_mission_index]
      {
        // Takeoff completed
        navigateToPreviousWaypoint(handler, on_completed, current_mission_index);
      });
  } else {
    RCLCPP_WARN(
      _node.get_logger(),
      "Resume: no takeoff action found in current mission, continuing directly");
    on_completed();
  }
}

void OnResume::navigateToPreviousWaypoint(
  const std::shared_ptr<ActionHandler> & handler,
  const std::function<void()> & on_completed, int current_mission_index)
{
  RCLCPP_DEBUG(_node.get_logger(), "Resume: navigating to waypoint before the current one");
  bool trajectory_run = false;
  for (int index =
    std::min(current_mission_index, static_cast<int>(handler->mission().items().size())) - 1;
    index >= 0 && !trajectory_run; --index)
  {
    if (const auto * navigation_item =
      std::get_if<NavigationItem>(&handler->mission().items()[index]))
    {
      std::visit(
        util::Overloaded{
          [&](const Waypoint & waypoint)
          {
            switch (waypoint.frame) {
              case MissionFrame::Global:
                const auto mission = std::make_shared<Mission>(
                  std::vector<MissionItem>{waypoint});
                handler->runTrajectory(mission, on_completed, false);
                trajectory_run = true;
                break;
            }
          }
        }, navigation_item->data);
    }
  }
  if (!trajectory_run) {
    // No waypoint found, continue directly
    on_completed();
  }
}

void OnFailure::run(
  const std::shared_ptr<ActionHandler> & handler, const ActionArguments & arguments,
  const std::function<void()> & on_completed)
{
  // Try RTL first, then fall back to Descend
  handler->runMode(
    ModeBase::kModeIDRtl, on_completed, [handler, on_completed]
    {
      handler->runMode(
        ModeBase::kModeIDDescend, on_completed, [handler, on_completed]
        {
          // If this one fails, do nothing
        });

    });
}

void ChangeSettings::run(
  const std::shared_ptr<ActionHandler> & handler, const ActionArguments & arguments,
  const std::function<void()> & on_completed)
{
  if (arguments.contains("resetAll") && arguments.at<bool>("resetAll")) {
    handler->clearTrajectoryOptions();
    _state.reset();
  } else {
    // Merge with existing settings
    auto options = handler->getTrajectoryOptions();
    if (arguments.contains("horizontalVelocity")) {
      options.horizontal_velocity = arguments.at<float>("horizontalVelocity");
    }
    if (arguments.contains("verticalVelocity")) {
      options.vertical_velocity = arguments.at<float>("verticalVelocity");
    }
    if (arguments.contains("maxHeadingRate")) {
      options.max_heading_rate = degToRad(arguments.at<float>("maxHeadingRate"));
    }
    handler->setTrajectoryOptions(options);
    _state = handler->storeState(name(), ActionArguments(nlohmann::json(options)));
  }
  on_completed();
}

void ChangeSettings::deactivate()
{
  // We don't need to reset the settings, just clear the state
  _state.reset();
}
}
