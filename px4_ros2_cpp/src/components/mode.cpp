/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "px4_ros2/components/mode.hpp"
#include "px4_ros2/components/message_compatibility_check.hpp"
#include "px4_ros2/components/wait_for_fmu.hpp"
#include "px4_ros2/utils/message_version.hpp"

#include "registration.hpp"

#include <cassert>
#include <cfloat>
#include <utility>

namespace px4_ros2
{

ModeBase::ModeBase(
  rclcpp::Node & node, ModeBase::Settings settings, const std::string & topic_namespace_prefix)
: Context(node, topic_namespace_prefix),
  _registration(std::make_shared<Registration>(node, topic_namespace_prefix)),
  _settings(std::move(settings)),
  _health_and_arming_checks(node,
    [this](auto && reporter) {
      checkArmingAndRunConditions(std::forward<decltype(reporter)>(reporter));
    },
    topic_namespace_prefix), _config_overrides(node, topic_namespace_prefix)
{
  _vehicle_status_sub = node.create_subscription<px4_msgs::msg::VehicleStatus>(
    topic_namespace_prefix + "fmu/out/vehicle_status" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(), rclcpp::QoS(
      1).best_effort(),
    [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
      if (_registration->registered()) {
        vehicleStatusUpdated(msg);
      }
    });
  _mode_completed_pub = node.create_publisher<px4_msgs::msg::ModeCompleted>(
    topic_namespace_prefix + "fmu/in/mode_completed" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::ModeCompleted>(),
    1);
  _config_control_setpoints_pub = node.create_publisher<px4_msgs::msg::VehicleControlMode>(
    topic_namespace_prefix + "fmu/in/config_control_setpoints" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleControlMode>(),
    1);
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

  if (!_skip_message_compatibility_check && (!waitForFMU(node(), 15s) ||
    !messageCompatibilityCheck(node(), {ALL_PX4_ROS2_MESSAGES}, topicNamespacePrefix())))
  {
    return false;
  }

  onAboutToRegister();

  _health_and_arming_checks.overrideRegistration(_registration);
  const RegistrationSettings settings = getRegistrationSettings();
  bool ret = _registration->doRegister(settings);

  if (ret) {
    if (!onRegistered()) {
      ret = false;
    }
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
  RCLCPP_DEBUG(node().get_logger(), "Mode '%s' activated", _registration->name().c_str());
  _is_active = true;
  _completed = false;
  _last_setpoint_update = node().get_clock()->now();
  onActivate();

  if (_setpoint_update_rate_hz > FLT_EPSILON) {
    updateSetpoint(1.f / _setpoint_update_rate_hz);             // Immediately update
  }

  updateSetpointUpdateTimer();
}

void ModeBase::callOnDeactivate()
{
  RCLCPP_DEBUG(node().get_logger(), "Mode '%s' deactivated", _registration->name().c_str());
  _is_active = false;
  onDeactivate();
  updateSetpointUpdateTimer();
}

void ModeBase::updateSetpointUpdateTimer()
{
  const bool activate = _is_active && _setpoint_update_rate_hz > FLT_EPSILON;

  if (activate) {
    if (!_setpoint_update_timer) {
      _setpoint_update_timer = node().create_wall_timer(
        std::chrono::milliseconds(
          static_cast<int64_t>(1000.f /
          _setpoint_update_rate_hz)), [this]() {
          const auto now = node().get_clock()->now();
          const float dt_s = (now - _last_setpoint_update).seconds();
          _last_setpoint_update = now;
          updateSetpoint(dt_s);
        });
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
      node().get_logger(), "Mode '%s': completed was already called",
      _registration->name().c_str());
    return;
  }

  px4_msgs::msg::ModeCompleted mode_completed{};
  mode_completed.nav_state = static_cast<uint8_t>(id());
  mode_completed.result = static_cast<uint8_t>(result);
  mode_completed.timestamp = 0; // Let PX4 set the timestamp
  _mode_completed_pub->publish(mode_completed);
  _completed = true;
}

void ModeBase::onAboutToRegister()
{
  // Move _new_setpoint_types to _setpoint_types and register activation callback
  assert(_setpoint_types.empty());
  for (auto * setpoint : _new_setpoint_types) {
    _setpoint_types.push_back(setpoint->getSharedPtr());

    setpoint->setShouldActivateCallback(
      [this, setpoint]() {
        for (auto & setpoint_type : _setpoint_types) {
          if (setpoint_type.get() == setpoint) {
            activateSetpointType(*setpoint);
            RCLCPP_DEBUG(
              node().get_logger(), "Mode '%s': changing setpoint type",
              _registration->name().c_str());
          } else {
            setpoint_type->setActive(false);
          }
        }
      });
  }
  _new_setpoint_types.clear();

  updateModeRequirementsFromSetpoints();
}

bool ModeBase::onRegistered()
{
  _config_overrides.setup(
    px4_msgs::msg::ConfigOverrides::SOURCE_TYPE_MODE,
    _registration->modeId());

  if (_setpoint_types.empty()) {
    RCLCPP_FATAL(node().get_logger(), "At least one setpoint type must be created");
    return false;
  }

  // TODO: check setpoint types compatibility with current vehicle type

  activateSetpointType(*_setpoint_types[0]);
  if (_setpoint_update_rate_hz < FLT_EPSILON) {
    // Do not use default setpoint rate if rate was already set by user
    setSetpointUpdateRateFromSetpointTypes();
  }

  return true;
}

void ModeBase::updateModeRequirementsFromSetpoints()
{
  // Set a mode requirement if at least one setypoint type requires it
  RequirementFlags & requirements = modeRequirements();
  for (const auto & setpoint_type : _setpoint_types) {
    const auto config = setpoint_type->getConfiguration();

    requirements.angular_velocity |= config.rates_enabled;
    requirements.attitude |= config.attitude_enabled;
    requirements.local_alt |= config.altitude_enabled;
    requirements.local_alt |= config.climb_rate_enabled;

    if (!config.local_position_is_optional) {
      requirements.local_position |= config.velocity_enabled;
      requirements.local_position |= config.position_enabled;
    }
  }

  if (requirements.manual_control) {
    // Use relaxed local position accuracy if a manual mode
    if (requirements.local_position) {
      requirements.local_position = false;
      requirements.local_position_relaxed = true;
    }
  }
}

void ModeBase::setSetpointUpdateRateFromSetpointTypes()
{
  // Set update rate based on setpoint types
  float max_update_rate = -1.f;
  for (const auto & setpoint_type : _setpoint_types) {
    if (setpoint_type->desiredUpdateRateHz() > max_update_rate) {
      max_update_rate = setpoint_type->desiredUpdateRateHz();
    }
  }
  if (max_update_rate > 0.f) {
    setSetpointUpdateRate(max_update_rate);
  }
}

void ModeBase::activateSetpointType(SetpointBase & setpoint)
{
  setpoint.setActive(true);
  px4_msgs::msg::VehicleControlMode control_mode{};
  control_mode.source_id = static_cast<uint8_t>(id());
  setpoint.getConfiguration().fillControlMode(control_mode);
  control_mode.timestamp = 0; // Let PX4 set the timestamp
  _config_control_setpoints_pub->publish(control_mode);
}

void ModeBase::addSetpointType(SetpointBase * setpoint)
{
  assert(!_registration->registered()); // enforce initialization before registration (i.e. in mode constructor)
  // setpoint is currently being constructed, so we cannot get a shared pointer to it, and we cannot call virtual
  // methods. So we just store the pointer for later use
  _new_setpoint_types.push_back(setpoint);
}

void ModeBase::setRequirement(const RequirementFlags & requirement_flags)
{
  assert(!_registration->registered()); // enforce initialization before registration (i.e. in mode constructor)
  modeRequirements() |= requirement_flags;
}

} // namespace px4_ros2
