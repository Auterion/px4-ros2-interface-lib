/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "registration.hpp"
#include "px4_ros2/components/health_and_arming_checks.hpp"
#include "px4_ros2/utils/message_version.hpp"

#include <cassert>
#include <utility>

using namespace std::chrono_literals;

namespace px4_ros2
{

HealthAndArmingChecks::HealthAndArmingChecks(
  rclcpp::Node & node, CheckCallback check_callback,
  const std::string & topic_namespace_prefix)
: _node(node), _registration(std::make_shared<Registration>(node, topic_namespace_prefix)),
  _check_callback(std::move(check_callback))
{
  _arming_check_reply_pub = _node.create_publisher<px4_msgs::msg::ArmingCheckReply>(
    topic_namespace_prefix + "fmu/in/arming_check_reply" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::ArmingCheckReply>(),
    1);

  _arming_check_request_sub = _node.create_subscription<px4_msgs::msg::ArmingCheckRequest>(
    topic_namespace_prefix + "fmu/out/arming_check_request" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::ArmingCheckRequest>(),
    rclcpp::QoS(1).best_effort(),
    [this](px4_msgs::msg::ArmingCheckRequest::UniquePtr msg) {

      RCLCPP_DEBUG_ONCE(
        _node.get_logger(), "Arming check request (id=%i, only printed once)",
        msg->request_id);

      if (_registration->registered()) {
        px4_msgs::msg::ArmingCheckReply reply{};
        reply.registration_id = _registration->armingCheckId();
        reply.request_id = msg->request_id;
        reply.can_arm_and_run = true;

        HealthAndArmingCheckReporter reporter(reply);
        _check_callback(reporter);

        _mode_requirements.fillArmingCheckReply(reply);

        reply.timestamp = 0; // Let PX4 set the timestamp
        _arming_check_reply_pub->publish(reply);
        _check_triggered = true;

      } else {
        RCLCPP_DEBUG(_node.get_logger(), "...not registered yet");
      }
    });

  _watchdog_timer =
    _node.create_wall_timer(4s, [this] {watchdogTimerUpdate();});
}

void HealthAndArmingChecks::overrideRegistration(const std::shared_ptr<Registration> & registration)
{
  assert(!_registration->registered());
  _registration = registration;
}

bool HealthAndArmingChecks::doRegister(const std::string & name)
{
  assert(!_registration->registered());
  RegistrationSettings settings{};
  settings.name = name;
  settings.register_arming_check = true;
  return _registration->doRegister(settings);
}

void HealthAndArmingChecks::watchdogTimerUpdate()
{
  if (_registration->registered()) {
    if (!_check_triggered && _shutdown_on_timeout) {
      rclcpp::shutdown();
      throw std::runtime_error(
              "Timeout, no request received from FMU, exiting (this can happen on FMU reboots)");
    }

    _check_triggered = false;

  } else {
    // avoid false positives while unregistered
    _check_triggered = true;
  }
}

} // namespace px4_ros2
