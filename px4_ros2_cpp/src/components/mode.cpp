/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "px4_ros2/components/mode.hpp"

#include <cassert>
#include <cfloat>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <utility>

#include "px4_ros2/components/message_compatibility_check.hpp"
#include "px4_ros2/components/wait_for_fmu.hpp"
#include "px4_ros2/utils/message_version.hpp"
#include "registration.hpp"

namespace px4_ros2 {

ModeBase::ModeBase(rclcpp::Node& node, ModeBase::Settings settings,
                   const std::string& topic_namespace_prefix)
    : Context(node, topic_namespace_prefix),
      _registration(std::make_shared<Registration>(node, topic_namespace_prefix)),
      _settings(std::move(settings)),
      _health_and_arming_checks(
          node,
          [this](auto&& reporter) {
            checkArmingAndRunConditions(std::forward<decltype(reporter)>(reporter));
          },
          topic_namespace_prefix),
      _config_overrides(node, topic_namespace_prefix)
{
  if (_settings.prevent_arming) {
    modeRequirements().prevent_arming = true;
  }
  // Use a globally shared vehicle status instance for consistent callback ordering when using
  // multiple modes/executors
  _vehicle_status_sub_cb = SharedSubscription<px4_msgs::msg::VehicleStatus>::create(
      node,
      topic_namespace_prefix + "fmu/out/vehicle_status" +
          px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(),
      [this](const px4_msgs::msg::VehicleStatus::UniquePtr& msg) {
        if (_registration->registered()) {
          vehicleStatusUpdated(msg);
        }
      });
  _mode_completed_pub = node.create_publisher<px4_msgs::msg::ModeCompleted>(
      topic_namespace_prefix + "fmu/in/mode_completed" +
          px4_ros2::getMessageNameVersion<px4_msgs::msg::ModeCompleted>(),
      1);
  _setpoint_config_pub = node.create_publisher<px4_msgs::msg::SetpointConfig>(
      topic_namespace_prefix + "fmu/in/setpoint_config" +
          px4_ros2::getMessageNameVersion<px4_msgs::msg::SetpointConfig>(),
      1);

  _setpoint_config_reply_sub_cb = SharedSubscription<px4_msgs::msg::SetpointConfigReply>::create(
      node,
      topic_namespace_prefix + "fmu/out/setpoint_config_reply" +
          px4_ros2::getMessageNameVersion<px4_msgs::msg::SetpointConfigReply>(),
      [this](const px4_msgs::msg::SetpointConfigReply::UniquePtr& msg) {
        if (msg->source_id == id()) {
          if (_current_activating_setpoint &&
              _current_activating_setpoint->getSetpointType() == msg->type) {
            _current_activating_setpoint->setActive(true);
            _current_activating_setpoint.reset();
          }
        }
      });
}

ModeBase::ModeID ModeBase::id() const
{
  return _registration->modeId();
}

void ModeBase::overrideRegistration(const std::shared_ptr<Registration>& registration)
{
  assert(!_registration->registered());
  _health_and_arming_checks.overrideRegistration(registration);
  _registration = registration;
}

// NOLINTNEXTLINE Cannot use default constructor due to incomplete type VehicleStatusSingletonToken
ModeBase::~ModeBase()
{
}

bool ModeBase::doRegister()
{
  assert(!_registration->registered());

  if (!_skip_message_compatibility_check && (!waitForFMU(node(), 15s, 5s, topicNamespacePrefix()) ||
                                             !defaultMessageCompatibilityCheck())) {
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

  settings.user_selectable = _settings.user_selectable;

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
    updateSetpoint(1.f / _setpoint_update_rate_hz);  // Immediately update
  }

  updateSetpointUpdateTimer();
}

void ModeBase::callOnDeactivate()
{
  RCLCPP_DEBUG(node().get_logger(), "Mode '%s' deactivated", _registration->name().c_str());
  _is_active = false;
  deactivateAllSetpointTypes();
  onDeactivate();
  updateSetpointUpdateTimer();
}

void ModeBase::updateSetpointUpdateTimer()
{
  const bool activate = _is_active && _setpoint_update_rate_hz > FLT_EPSILON;

  if (activate) {
    if (!_setpoint_update_timer) {
      // Use node clock for sim time support
      _setpoint_update_timer = rclcpp::create_timer(
          &node(), node().get_clock(),
          rclcpp::Duration::from_seconds(1.0 / _setpoint_update_rate_hz), [this]() {
            const auto now = node().get_clock()->now();
            const float dt_s = (now - _last_setpoint_update).seconds();
            _last_setpoint_update = now;
            updateSetpoint(dt_s);
          });
    }

  } else {
    if (_setpoint_update_timer) {
      _setpoint_update_timer = nullptr;
    }
  }
}

void ModeBase::checkSetpointCompatibilityAndRequirements()
{
  // Check setpoint types compatibility with current vehicle type

  // Create a fresh subscription to avoid ROS Jazzy WaitSet conflicts
  const auto setpoint_config_reply_sub =
      node().create_subscription<px4_msgs::msg::SetpointConfigReply>(
          topicNamespacePrefix() + "fmu/out/setpoint_config_reply" +
              px4_ros2::getMessageNameVersion<px4_msgs::msg::SetpointConfigReply>(),
          rclcpp::QoS(1).best_effort(), [](px4_msgs::msg::SetpointConfigReply::UniquePtr) {});

  // Wait until DDS discovery has matched both directions so that the very
  // first publish is not silently dropped
  const auto discovery_start = std::chrono::steady_clock::now();
  const auto discovery_timeout = 3000ms;
  while (setpoint_config_reply_sub->get_publisher_count() == 0 ||
         _setpoint_config_pub->get_subscription_count() == 0) {
    if (std::chrono::steady_clock::now() >= discovery_start + discovery_timeout) {
      RCLCPP_WARN(node().get_logger(),
                  "Timeout waiting for setpoint config discovery "
                  "(reply publishers=%zu, config subscribers=%zu)",
                  setpoint_config_reply_sub->get_publisher_count(),
                  _setpoint_config_pub->get_subscription_count());
      break;
    }
    std::this_thread::sleep_for(50ms);
  }
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(setpoint_config_reply_sub);

  unsigned setpoint_index = 0;
  for (const auto& setpoint : _setpoint_types) {
    px4_msgs::msg::SetpointConfig setpoint_config{};
    setpoint_config.source_id = static_cast<uint8_t>(id());
    setpoint_config.should_apply = false;
    setpoint_config.type = setpoint->getSetpointType();
    setpoint_config.timestamp = 0;  // Let PX4 set the timestamp

    bool got_reply = false;

    for (int retries = 0; retries < 5 && !got_reply; ++retries) {
      _setpoint_config_pub->publish(setpoint_config);
      auto start_time = std::chrono::steady_clock::now();
      const auto timeout = 300ms;
      while (!got_reply) {
        auto now = std::chrono::steady_clock::now();

        if (now >= start_time + timeout) {
          break;
        }

        auto wait_ret = wait_set.wait(timeout - (now - start_time));

        if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
          px4_msgs::msg::SetpointConfigReply reply;
          rclcpp::MessageInfo info;

          if (setpoint_config_reply_sub->take(reply, info)) {
            if (reply.source_id == id() && reply.type == setpoint_config.type) {
              if (reply.result != px4_msgs::msg::SetpointConfigReply::RESULT_SUCCESS) {
                // This is fatal, the setpoint cannot be used.
                // We could extend the API and allow for optional setpoint types, so that a mode
                // could fall back to another type (to e.g. support multiple vehicle types).
                switch (reply.result) {
                  case px4_msgs::msg::SetpointConfigReply::RESULT_UNSUPPORTED:
                    throw Exception("Setpoint type " + std::to_string(setpoint_config.type) +
                                    " with index " + std::to_string(setpoint_index) +
                                    " is not supported by the current vehicle type");
                  case px4_msgs::msg::SetpointConfigReply::RESULT_UNKNOWN_SETPOINT_TYPE:
                    throw Exception("Setpoint type " + std::to_string(setpoint_config.type) +
                                    " with index " + std::to_string(setpoint_index) +
                                    " is not known by the FMU");
                  case px4_msgs::msg::SetpointConfigReply::RESULT_FAILURE_OTHER:
                  default:
                    throw Exception("Setpoint type " + std::to_string(setpoint_config.type) +
                                    " with index " + std::to_string(setpoint_index) +
                                    " was rejected by the FMU");
                }
              }

              // Apply mode requirement flags
              setpoint->clearOptionalRequirements(reply);
              RequirementFlags& requirements = modeRequirements();
              requirements.angular_velocity |= reply.mode_req_angular_velocity;
              requirements.attitude |= reply.mode_req_attitude;
              requirements.local_alt |= reply.mode_req_local_alt;
              requirements.local_position |= reply.mode_req_local_position;

              if (requirements.manual_control) {
                // Use relaxed local position accuracy if a manual mode
                if (requirements.local_position) {
                  requirements.local_position = false;
                  requirements.local_position_relaxed = true;
                }
              }

              got_reply = true;
            }
          } else {
            RCLCPP_DEBUG(node().get_logger(), "No SetpointConfigReply message received");
          }

        } else {
          RCLCPP_DEBUG(node().get_logger(), "timeout");
        }
      }
    }

    if (!got_reply) {
      // If we did not get a reply, something is very wrong
      throw Exception("Did not get a reply from FMU for setpoint configuration");
    }
    ++setpoint_index;
  }
  wait_set.remove_subscription(setpoint_config_reply_sub);
}

void ModeBase::setSetpointUpdateRate(float rate_hz)
{
  _setpoint_update_timer = nullptr;
  _setpoint_update_rate_hz = rate_hz;
  updateSetpointUpdateTimer();
}

void ModeBase::unsubscribeVehicleStatus()
{
  _vehicle_status_sub_cb.reset();
}

void ModeBase::vehicleStatusUpdated(const px4_msgs::msg::VehicleStatus::UniquePtr& msg,
                                    bool do_not_activate)
{
  // Update state
  _is_armed = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  const bool is_active =
      id() == msg->nav_state && (_is_armed || _settings.activate_even_while_disarmed);

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
    RCLCPP_DEBUG_ONCE(node().get_logger(), "Mode '%s': completed was already called",
                      _registration->name().c_str());
    // Continue to publish the topic, in case the previous one got lost
  }

  px4_msgs::msg::ModeCompleted mode_completed{};
  mode_completed.nav_state = static_cast<uint8_t>(id());
  mode_completed.result = static_cast<uint8_t>(result);
  mode_completed.timestamp = 0;  // Let PX4 set the timestamp
  _mode_completed_pub->publish(mode_completed);
  _completed = true;
}

void ModeBase::onAboutToRegister()
{
  // Move _new_setpoint_types to _setpoint_types and register activation callback
  assert(_setpoint_types.empty());
  for (auto* setpoint : _new_setpoint_types) {
    _setpoint_types.push_back(setpoint->getSharedPtr());

    setpoint->setShouldActivateCallback([this, setpoint]() {
      for (auto& setpoint_type : _setpoint_types) {
        if (setpoint_type.get() == setpoint) {
          activateSetpointType(setpoint_type);
          RCLCPP_DEBUG(node().get_logger(), "Mode '%s': changing setpoint type",
                       _registration->name().c_str());
        } else {
          setpoint_type->setActive(false);
        }
      }
    });
  }
  _new_setpoint_types.clear();
}

bool ModeBase::onRegistered()
{
  _config_overrides.setup(px4_msgs::msg::ConfigOverrides::SOURCE_TYPE_MODE,
                          _registration->modeId());

  if (_setpoint_types.empty()) {
    RCLCPP_FATAL(node().get_logger(), "At least one setpoint type must be created");
    return false;
  }

  checkSetpointCompatibilityAndRequirements();

  if (_setpoint_update_rate_hz < FLT_EPSILON) {
    // Do not use default setpoint rate if rate was already set by user
    setSetpointUpdateRateFromSetpointTypes();
  }

  return true;
}

void ModeBase::setSetpointUpdateRateFromSetpointTypes()
{
  // Set update rate based on setpoint types
  float max_update_rate = -1.f;
  for (const auto& setpoint_type : _setpoint_types) {
    max_update_rate = std::max(max_update_rate, setpoint_type->desiredUpdateRateHz());
  }
  if (max_update_rate > 0.f) {
    setSetpointUpdateRate(max_update_rate);
  }
}

void ModeBase::activateSetpointType(const std::shared_ptr<SetpointBase>& setpoint)
{
  _current_activating_setpoint = setpoint;
  px4_msgs::msg::SetpointConfig setpoint_config{};
  setpoint_config.source_id = static_cast<uint8_t>(id());
  setpoint_config.should_apply = true;
  setpoint_config.type = setpoint->getSetpointType();
  setpoint_config.timestamp = 0;  // Let PX4 set the timestamp
  _setpoint_config_pub->publish(setpoint_config);
  // setActive() will be called when we get a matching reply from PX4. If not (e.g. on message
  // drop), the next setpoint update triggers another setpoint activation request.
}

void ModeBase::deactivateAllSetpointTypes()
{
  for (auto& setpoint_type : _setpoint_types) {
    setpoint_type->setActive(false);
  }
}

bool ModeBase::defaultMessageCompatibilityCheck()
{
  // Call the compatibility check only once and store the result (in case there are multiple modes)
  static const bool kResult =
      messageCompatibilityCheck(node(), {ALL_PX4_ROS2_MESSAGES}, topicNamespacePrefix());
  return kResult;
}

void ModeBase::addSetpointType(SetpointBase* setpoint)
{
  assert(!_registration->registered());  // enforce initialization before registration (i.e. in mode
                                         // constructor)
  // setpoint is currently being constructed, so we cannot get a shared pointer to it, and we cannot
  // call virtual methods. So we just store the pointer for later use
  _new_setpoint_types.push_back(setpoint);
}

void ModeBase::setRequirement(const RequirementFlags& requirement_flags)
{
  assert(!_registration->registered());  // enforce initialization before registration (i.e. in mode
                                         // constructor)
  modeRequirements() |= requirement_flags;
}

}  // namespace px4_ros2
