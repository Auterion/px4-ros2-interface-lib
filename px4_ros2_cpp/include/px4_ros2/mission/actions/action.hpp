/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <px4_ros2/mission/mission.hpp>

namespace px4_ros2
{
/** \ingroup mission
 *  @{
 */

class ActionHandler;

/**
 * @brief Interface class for an action
 * @ingroup mission
 */
class ActionInterface
{
public:
  virtual ~ActionInterface() = default;

  virtual std::string name() const = 0;

  virtual bool canRun(const ActionArguments & arguments, std::vector<std::string> & errors)
  {
    return true;
  }

  /**
   * If this returns true, trajectory navigation stops the vehicle at the previous waypoint.
   */
  virtual bool shouldStopAtWaypoint(const ActionArguments & arguments) {return true;}

  /**
   * If the action includes a landing with disarming sequence, this method can be used to return true,
   * so that when activating the mission again, the action is directly called (w/o doing the mission takeoff first)
   * @return true if the action can be resumed from landed state
   */
  virtual bool supportsResumeFromLanded() {return false;}

  virtual void run(
    const std::shared_ptr<ActionHandler> & handler,
    const ActionArguments & arguments,
    const std::function<void()> & on_completed) = 0;

  /**
   * This is called when the mission gets deactivated, i.e., the user switches to a different mode, or the mission
   * finished. It can be used to disable continuous actions.
   */
  virtual void deactivate() {}
};


/** @}*/
} /* namespace px4_ros2 */
