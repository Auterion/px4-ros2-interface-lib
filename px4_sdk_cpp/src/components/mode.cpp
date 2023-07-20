/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "px4_sdk/components/mode.hpp"
#include "px4_sdk/components/message_compatibility_check.hpp"

#include "registration.hpp"

#include <cassert>
#include <cfloat>
#include <utility>

namespace px4_sdk
{

ModeBase::ModeBase(
  rclcpp::Node & node, ModeBase::Settings settings,
  const ModeRequirements & requirements, const std::string & topic_namespace_prefix)
: _node(node), _topic_namespace_prefix(topic_namespace_prefix),
  _registration(std::make_shared<Registration>(node, topic_namespace_prefix)),
  _settings(std::move(settings)),
  _health_and_arming_checks(node,
    [this](auto && reporter) {
      checkArmingAndRunConditions(std::forward<decltype(reporter)>(reporter));
    },
    topic_namespace_prefix),
  _setpoint_sender(node, *this, topic_namespace_prefix), _config_overrides(
    node,
    topic_namespace_prefix)
{
  _vehicle_status_sub = _node.create_subscription<px4_msgs::msg::VehicleStatus>(
    topic_namespace_prefix + "/fmu/out/vehicle_status", rclcpp::QoS(1).best_effort(),
    [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
      if (_registration->registered()) {
        vehicleStatusUpdated(msg);
      }
    });
  _mode_completed_pub = _node.create_publisher<px4_msgs::msg::ModeCompleted>(
    topic_namespace_prefix + "/fmu/in/mode_completed", 1);

  _health_and_arming_checks.setModeRequirements(requirements);
}

ModeBase::ModeID ModeBase::id() const
{
  return _registration->modeId();
}

void ModeBase::overrideRegistration(const std::shared_ptr<Registration> & registration)
{
  assert(!_registration->registered());
  _health_and_arming_checks.overrideRegistration(registration);
  _registration = registration;
}

bool ModeBase::doRegister()
{
  assert(!_registration->registered());

  if (!messageCompatibilityCheck(_node, {ALL_PX4_SDK_MESSAGES}, _topic_namespace_prefix)) {
    return false;
  }

  _health_and_arming_checks.overrideRegistration(_registration);
  const RegistrationSettings settings = getRegistrationSettings();
  const bool ret = _registration->doRegister(settings);

  if (ret) {
    onRegistered();
  }

  return ret;
}

RegistrationSettings ModeBase::getRegistrationSettings() const
{
  RegistrationSettings settings{};
  settings.name = _settings.name;
  settings.register_arming_check = true;
  settings.register_mode = true;

  if (_settings.replace_internal_mode != kModeIDInvalid) {
    settings.enable_replace_internal_mode = true;
    settings.replace_internal_mode = _settings.replace_internal_mode;
  }

  return settings;
}

void ModeBase::callOnActivate()
{
  RCLCPP_DEBUG(_node.get_logger(), "Mode '%s' activated", _registration->name().c_str());
  _is_active = true;
  _completed = false;
  onActivate();

  if (_setpoint_update_rate_hz > FLT_EPSILON) {
    updateSetpoint();             // Immediately update
  }

  updateSetpointUpdateTimer();
}

void ModeBase::callOnDeactivate()
{
  RCLCPP_DEBUG(_node.get_logger(), "Mode '%s' deactivated", _registration->name().c_str());
  _is_active = false;
  onDeactivate();
  updateSetpointUpdateTimer();
}

void ModeBase::updateSetpointUpdateTimer()
{
  const bool activate = _is_active && _setpoint_update_rate_hz > FLT_EPSILON;

  if (activate) {
    if (!_setpoint_update_timer) {
      _setpoint_update_timer = _node.create_wall_timer(
        std::chrono::milliseconds(
          static_cast<int64_t>(1000.F /
          _setpoint_update_rate_hz)), [this]() {updateSetpoint();});
    }

  } else {
    if (_setpoint_update_timer) {
      _setpoint_update_timer.reset();
    }
  }
}

void ModeBase::setSetpointUpdateRate(float rate_hz)
{
  _setpoint_update_timer.reset();
  _setpoint_update_rate_hz = rate_hz;
  updateSetpointUpdateTimer();
}

void ModeBase::unsubscribeVehicleStatus()
{
  _vehicle_status_sub.reset();
}

void ModeBase::vehicleStatusUpdated(
  const px4_msgs::msg::VehicleStatus::UniquePtr & msg,
  bool do_not_activate)
{
  // Update state
  _is_armed = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  const bool is_active = id() == msg->nav_state &&
    (_is_armed || _settings.activate_even_while_disarmed);

  if (_is_active != is_active) {
    if (is_active) {
      if (!do_not_activate) {
        callOnActivate();
      }

    } else {
      callOnDeactivate();
    }
  }
}

void ModeBase::completed(Result result)
{
  if (_completed) {
    RCLCPP_DEBUG_ONCE(
      _node.get_logger(), "Mode '%s': completed was already called", _registration->name().c_str());
    return;
  }

  px4_msgs::msg::ModeCompleted mode_completed{};
  mode_completed.nav_state = static_cast<uint8_t>(id());
  mode_completed.result = static_cast<uint8_t>(result);
  mode_completed.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
  _mode_completed_pub->publish(mode_completed);
  _completed = true;
}

void ModeBase::onRegistered()
{
  _config_overrides.setup(
    px4_msgs::msg::ConfigOverrides::SOURCE_TYPE_MODE,
    _registration->modeId());
}

} // namespace px4_sdk
