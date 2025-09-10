/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_ros2/mission/actions/action.hpp>
#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/vehicle_state/land_detected.hpp>
#include <rclcpp/rclcpp.hpp>


namespace px4_ros2::default_actions
{
/** \ingroup mission_default_actions
 *  @{
 */

/**
 * @brief Default action to trigger the RTL mode
 * @ingroup mission_default_actions
 */
class Rtl : public ActionInterface
{
public:
  ~Rtl() override = default;
  std::string name() const override {return "rtl";}

  bool shouldStopAtWaypoint(const ActionArguments & arguments) override {return false;}
  void run(
    const std::shared_ptr<ActionHandler> & handler, const ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    handler->runMode(ModeBase::kModeIDRtl, on_completed);
  }
};

/**
 * @brief Default action to trigger the Land mode
 * @ingroup mission_default_actions
 */
class Land : public ActionInterface
{
public:
  ~Land() override = default;
  std::string name() const override {return "land";}

  bool shouldStopAtWaypoint(const ActionArguments & arguments) override {return true;}
  void run(
    const std::shared_ptr<ActionHandler> & handler, const ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    handler->runMode(ModeBase::kModeIDLand, on_completed);
  }
};

/**
 * @brief Default action to trigger the Takeoff mode
 * @ingroup mission_default_actions
 */
class Takeoff : public ActionInterface
{
public:
  ~Takeoff() override = default;
  std::string name() const override {return "takeoff";}

  void run(
    const std::shared_ptr<ActionHandler> & handler, const ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    handler->runMode(ModeBase::kModeIDTakeoff, on_completed);
  }
};

/**
 * @brief Default action to trigger the Hold (Loiter) mode
 * @ingroup mission_default_actions
 */
class Hold : public ActionInterface
{
public:
  explicit Hold(ModeBase & mode)
  : _node(mode.node())
  {
  }
  ~Hold() override = default;
  std::string name() const override {return "hold";}

  bool shouldStopAtWaypoint(const ActionArguments & arguments) override {return true;}

  void run(
    const std::shared_ptr<ActionHandler> & handler, const ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    float delay_s = 1.0f;
    if (arguments.contains("delay")) {
      delay_s = arguments.at<float>("delay");
    }
    handler->runMode(ModeBase::kModeIDLoiter, [] {});
    _timer = _node.create_wall_timer(
      std::chrono::duration<float>(delay_s), [this, on_completed] {
        _timer.reset();
        on_completed();
      });
  }

  void deactivate() override
  {
    _timer.reset();
  }

private:
  rclcpp::Node & _node;
  std::shared_ptr<rclcpp::TimerBase> _timer;
};

/**
 * @brief Default action to resume missions
 * @ingroup mission_default_actions
 */
class OnResume : public ActionInterface
{
public:
  explicit OnResume(ModeBase & mode, std::shared_ptr<LandDetected> land_detected)
  : _node(mode.node()), _landed(std::move(land_detected))
  {
  }

  ~OnResume() override = default;
  std::string name() const override {return "onResume";}

  void run(
    const std::shared_ptr<ActionHandler> & handler, const ActionArguments & arguments,
    const std::function<void()> & on_completed) override;

private:
  void resumeFromUnexpectedLanding(
    const std::shared_ptr<ActionHandler> & handler,
    const std::function<void()> & on_completed, int current_mission_index);
  void navigateToPreviousWaypoint(
    const std::shared_ptr<ActionHandler> & handler,
    const std::function<void()> & on_completed, int current_mission_index);

  rclcpp::Node & _node;
  std::shared_ptr<LandDetected> _landed;
};

/**
 * @brief Default action to handle failures
 * @ingroup mission_default_actions
 */
class OnFailure : public ActionInterface
{
public:
  ~OnFailure() override = default;
  std::string name() const override {return "onFailure";}

  void run(
    const std::shared_ptr<ActionHandler> & handler, const ActionArguments & arguments,
    const std::function<void()> & on_completed) override;
};

/**
 * @brief Action to change mission settings (e.g. velocity)
 * @ingroup mission_default_actions
 */
class ChangeSettings : public ActionInterface
{
public:
  ~ChangeSettings() override = default;
  std::string name() const override {return "changeSettings";}

  bool shouldStopAtWaypoint(const ActionArguments & arguments) override {return false;}

  void run(
    const std::shared_ptr<ActionHandler> & handler, const ActionArguments & arguments,
    const std::function<void()> & on_completed) override;

  void deactivate() override;

private:
  std::unique_ptr<ActionStateKeeper> _state;
};

/** @}*/
} /* namespace px4_ros2::default_actions */
