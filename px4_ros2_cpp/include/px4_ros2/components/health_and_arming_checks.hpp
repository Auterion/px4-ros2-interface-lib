/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/arming_check_reply.hpp>
#include <px4_msgs/msg/arming_check_request.hpp>
#include <px4_ros2/common/requirement_flags.hpp>

#include "events.hpp"

#include <memory>
#include <functional>

class Registration;

namespace px4_ros2
{

class HealthAndArmingCheckReporter
{
public:
  explicit HealthAndArmingCheckReporter(px4_msgs::msg::ArmingCheckReply & arming_check_reply)
  : _arming_check_reply(arming_check_reply) {}

  template<typename ... Args>
  void armingCheckFailureExt(
    uint32_t event_id,
    events::Log log_level, const char * message, Args... args)
  {
    const uint16_t navigation_mode_groups{};
    const uint8_t health_component_index{};

    _arming_check_reply.can_arm_and_run = false;

    if (!addEvent(
        event_id, log_level, message, navigation_mode_groups,
        health_component_index, args ...))
    {
      printf("Error: too many events\n");
    }
  }

  void setHealth(uint8_t health_component_index, bool is_present, bool warning, bool error);

private:
  template<typename ... Args>
  bool addEvent(
    uint32_t event_id, const events::LogLevels & log_levels, const char * message,
    Args... args);

  px4_msgs::msg::ArmingCheckReply & _arming_check_reply;
};


template<typename ... Args>
bool HealthAndArmingCheckReporter::addEvent(
  uint32_t event_id, const events::LogLevels & log_levels,
  const char * message, Args... args)
{
  if (_arming_check_reply.num_events >= _arming_check_reply.events.size()) {
    return false;
  }

  events::EventType & e = _arming_check_reply.events[_arming_check_reply.num_events];
  e.log_levels =
    (static_cast<uint8_t>(log_levels.internal) << 4) | static_cast<uint8_t>(log_levels.external);
  e.id = event_id;
  static_assert(
    events::util::sizeofArguments(args ...) <= sizeof(e.arguments),
    "Too many arguments");
  events::util::fillEventArguments(static_cast<uint8_t *>(e.arguments.data()), args ...);
  ++_arming_check_reply.num_events;
  return true;
}


inline void HealthAndArmingCheckReporter::setHealth(
  uint8_t health_component_index, bool is_present, bool warning,
  bool error)
{
  _arming_check_reply.health_component_index = health_component_index;
  _arming_check_reply.health_component_is_present = is_present;
  _arming_check_reply.health_component_warning = warning;
  _arming_check_reply.health_component_error = error;
}


class HealthAndArmingChecks
{
public:
  using CheckCallback = std::function<void (HealthAndArmingCheckReporter &)>;

  HealthAndArmingChecks(
    rclcpp::Node & node, CheckCallback check_callback,
    const std::string & topic_namespace_prefix = "");
  HealthAndArmingChecks(const HealthAndArmingChecks &) = delete;

  /**
   * Register the checks. Call this once on startup. This is a blocking method.
   * @param name registration name. Should uniquely identify the component with length < 25 characters
   * @return true on success
   */
  bool doRegister(const std::string & name);

  void setModeRequirements(const RequirementFlags & mode_requirements)
  {
    _mode_requirements = mode_requirements;
  }

  RequirementFlags & modeRequirements() {return _mode_requirements;}

private:
  friend class ModeBase;
  friend class ModeExecutorBase;
  void overrideRegistration(const std::shared_ptr<Registration> & registration);

  void watchdogTimerUpdate();

  rclcpp::Node & _node;
  std::shared_ptr<Registration> _registration;
  CheckCallback _check_callback;
  bool _check_triggered{true};

  rclcpp::Subscription<px4_msgs::msg::ArmingCheckRequest>::SharedPtr _arming_check_request_sub;
  rclcpp::Publisher<px4_msgs::msg::ArmingCheckReply>::SharedPtr _arming_check_reply_pub;

  RequirementFlags _mode_requirements{};
  rclcpp::TimerBase::SharedPtr _watchdog_timer;
  bool _shutdown_on_timeout{true};
};

} // namespace px4_ros2
