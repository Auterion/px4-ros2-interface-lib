/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "px4_ros2/components/mode_executor.hpp"
#include "px4_ros2/components/message_compatibility_check.hpp"
#include "px4_ros2/components/wait_for_fmu.hpp"
#include "px4_ros2/utils/message_version.hpp"

#include "registration.hpp"

#include <cassert>
#include <future>
using namespace std::chrono_literals;

namespace px4_ros2
{

ModeExecutorBase::ModeExecutorBase(
  rclcpp::Node & node, const ModeExecutorBase::Settings & settings,
  ModeBase & owned_mode, const std::string & topic_namespace_prefix)
: _node(node), _topic_namespace_prefix(topic_namespace_prefix), _settings(settings), _owned_mode(
    owned_mode),
  _registration(std::make_shared<Registration>(node, topic_namespace_prefix)),
  _current_scheduled_mode(node, topic_namespace_prefix),
  _config_overrides(node, topic_namespace_prefix)
{
  _vehicle_status_sub = _node.create_subscription<px4_msgs::msg::VehicleStatus>(
    topic_namespace_prefix + "fmu/out/vehicle_status" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(), rclcpp::QoS(
      1).best_effort(),
    [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
      if (_registration->registered()) {
        vehicleStatusUpdated(msg);
      }
    });

  _vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>(
    topic_namespace_prefix + "fmu/in/vehicle_command_mode_executor" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommand>(),
    1);

}

bool ModeExecutorBase::doRegister()
{
  if (_owned_mode._registration->registered()) {
    RCLCPP_FATAL(
      _node.get_logger(), "Mode executor %s: mode already registered",
      _registration->name().c_str());
  }

  assert(!_registration->registered());

  if (!waitForFMU(node(), 15s) ||
    !messageCompatibilityCheck(node(), {ALL_PX4_ROS2_MESSAGES}, _topic_namespace_prefix))
  {
    return false;
  }

  _owned_mode.onAboutToRegister();
  _owned_mode.overrideRegistration(_registration);
  _owned_mode.unsubscribeVehicleStatus();
  RegistrationSettings settings = _owned_mode.getRegistrationSettings();
  settings.register_mode_executor = true;
  settings.activate_mode_immediately =
    (_settings.activation == Settings::Activation::ActivateImmediately);
  bool ret = _registration->doRegister(settings);

  if (ret) {
    if (!_owned_mode.onRegistered()) {
      ret = false;
    }
    onRegistered();
  }

  return ret;
}

void ModeExecutorBase::onRegistered()
{
  _config_overrides.setup(
    px4_msgs::msg::ConfigOverrides::SOURCE_TYPE_MODE_EXECUTOR,
    _registration->modeExecutorId());
}

void ModeExecutorBase::callOnActivate()
{
  RCLCPP_DEBUG(_node.get_logger(), "Mode executor '%s' activated", _registration->name().c_str());
  _is_in_charge = true;
  onActivate();
}

void ModeExecutorBase::callOnDeactivate(DeactivateReason reason)
{
  RCLCPP_DEBUG(
    _node.get_logger(), "Mode executor '%s' deactivated (%i)",
    _registration->name().c_str(), (int)reason);
  _current_scheduled_mode.cancel();
  _current_wait_vehicle_status.cancel();
  _is_in_charge = false;
  _was_never_activated = false;       // Set on deactivation, so we stay activated for the first time (while disarmed)
  onDeactivate(reason);
}

int ModeExecutorBase::id() const
{
  return _registration->modeExecutorId();
}

Result ModeExecutorBase::sendCommandSync(
  uint32_t command, float param1, float param2, float param3, float param4,
  float param5, float param6, float param7)
{
  // Send command and wait for ack
  Result result{Result::Rejected};
  px4_msgs::msg::VehicleCommand cmd{};
  cmd.command = command;
  cmd.param1 = param1;
  cmd.param2 = param2;
  cmd.param3 = param3;
  cmd.param4 = param4;
  cmd.param5 = param5;
  cmd.param6 = param6;
  cmd.param7 = param7;
  cmd.source_component = px4_msgs::msg::VehicleCommand::COMPONENT_MODE_EXECUTOR_START + id();
  cmd.timestamp = 0; // Let PX4 set the timestamp

  // Create a new subscription here instead of in the ModeExecutorBase constructor, because
  // ROS Jazzy would throw an exception 'subscription already associated with a wait set'
  // (We could also use exchange_in_use_by_wait_set_state(), but that might cause an
  // inconsistent state)
  const auto vehicle_command_ack_sub = _node.create_subscription<px4_msgs::msg::VehicleCommandAck>(
    _topic_namespace_prefix + "fmu/out/vehicle_command_ack" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommandAck>(), rclcpp::QoS(
      1).best_effort(),
    [](px4_msgs::msg::VehicleCommandAck::UniquePtr msg) {});

  // Wait until we have a publisher
  auto start_time = std::chrono::steady_clock::now();
  while (vehicle_command_ack_sub->get_publisher_count() == 0) {
    const auto timeout = 3000ms;
    const auto now = std::chrono::steady_clock::now();
    if (now >= start_time + timeout) {
      RCLCPP_WARN(_node.get_logger(), "Timeout waiting for vehicle_command_ack publisher");
      break;
    }
  }

  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(vehicle_command_ack_sub);

  bool got_reply = false;

  for (int i = 0; i < 3 && !got_reply; ++i) {
    _vehicle_command_pub->publish(cmd);
    start_time = std::chrono::steady_clock::now();
    const auto timeout = 300ms;
    while (!got_reply) {
      auto now = std::chrono::steady_clock::now();

      if (now >= start_time + timeout) {
        break;
      }

      auto wait_ret = wait_set.wait(timeout - (now - start_time));

      if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
        px4_msgs::msg::VehicleCommandAck ack;
        rclcpp::MessageInfo info;

        if (vehicle_command_ack_sub->take(ack, info)) {
          if (ack.command == cmd.command && ack.target_component == cmd.
            source_component)
          {
            if (ack.result ==
              px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
            {
              result = Result::Success;
            }

            got_reply = true;
          }

        } else {
          RCLCPP_DEBUG(
            _node.get_logger(),
            "No VehicleCommandAck message received");
        }

      } else {
        RCLCPP_DEBUG(_node.get_logger(), "timeout");
      }
    }
  }

  wait_set.remove_subscription(vehicle_command_ack_sub);

  if (!got_reply) {
    // We don't expect to run into an ack timeout
    result = Result::Timeout;
    RCLCPP_WARN(
      _node.get_logger(), "Cmd %i: timeout, no ack received",
      cmd.command);
  }

  return result;
}

void ModeExecutorBase::scheduleMode(
  ModeBase::ModeID mode_id,
  const CompletedCallback & on_completed, bool forced)
{
  px4_msgs::msg::VehicleCommand cmd{};
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE;
  cmd.param1 = mode_id;
  scheduleMode(mode_id, cmd, on_completed, forced);
}

void ModeExecutorBase::scheduleMode(
  ModeBase::ModeID mode_id, const px4_msgs::msg::VehicleCommand & cmd,
  const CompletedCallback & on_completed, bool forced)
{
  if (!_is_armed && !forced) {
    on_completed(Result::Rejected);
    return;
  }

  // If there's already an active mode, cancel it (it will call the callback with a failure result)
  _current_scheduled_mode.cancel();

  const Result result = sendCommandSync(
    cmd.command, cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.param5, cmd.param6,
    cmd.param7);

  if (result != Result::Success) {
    on_completed(result);
    return;
  }

  // Store the callback and ensure it's called eventually. There's a number of outcomes:
  // - The mode finishes and publishes the completion result.
  // - Failsafe is entered or the user switches out. In that case the executor gets deactivated.
  // - The user switches into the owned mode. In that case the fmu does not deactivate the executor.
  _current_scheduled_mode.activate(mode_id, on_completed);
}

void ModeExecutorBase::takeoff(
  const CompletedCallback & on_completed, float altitude,
  float heading)
{
  px4_msgs::msg::VehicleCommand cmd{};
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
  cmd.param1 = NAN;
  cmd.param2 = NAN;
  cmd.param3 = NAN;
  cmd.param4 = heading;
  cmd.param5 = NAN;
  cmd.param6 = NAN;
  cmd.param7 = altitude;       // TODO: this is AMSL, local (relative to home) would be better
  scheduleMode(ModeBase::kModeIDTakeoff, cmd, on_completed);
}

void ModeExecutorBase::land(const CompletedCallback & on_completed)
{
  scheduleMode(ModeBase::kModeIDLand, on_completed);
}

void ModeExecutorBase::rtl(const CompletedCallback & on_completed)
{
  scheduleMode(ModeBase::kModeIDRtl, on_completed);
}

void ModeExecutorBase::arm(const CompletedCallback & on_completed, bool run_preflight_checks)
{
  if (_is_armed) {
    on_completed(Result::Success);
    return;
  }

  const float param2 = run_preflight_checks ? NAN : 21196.f;
  const Result result = sendCommandSync(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    1.f, param2);

  if (result != Result::Success) {
    on_completed(result);
    return;
  }

  // Wait until our internal state changes to armed
  _current_wait_vehicle_status.activate(
    [this](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) {return _is_armed;}, on_completed);
}

void ModeExecutorBase::disarm(const CompletedCallback & on_completed, bool forced)
{
  if (!_is_armed) {
    on_completed(Result::Success);
    return;
  }

  const float param2 = forced ? 21196.f : NAN;
  const Result result =
    sendCommandSync(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    0.f, param2);


  if (result != Result::Success) {
    on_completed(result);
    return;
  }

  // Wait until our internal state changes to disarmed
  _current_wait_vehicle_status.activate(
    [this](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) {return !_is_armed;}, on_completed);
}

void ModeExecutorBase::waitReadyToArm(const CompletedCallback & on_completed)
{
  if (_is_armed) {
    on_completed(Result::Success);
    return;
  }

  RCLCPP_DEBUG(_node.get_logger(), "Waiting until ready to arm...");
  _current_wait_vehicle_status.activate(
    [](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) {return msg->pre_flight_checks_pass;},
    on_completed);
}

void ModeExecutorBase::waitUntilDisarmed(const CompletedCallback & on_completed)
{
  if (!_is_armed) {
    on_completed(Result::Success);
    return;
  }

  RCLCPP_DEBUG(_node.get_logger(), "Waiting until disarmed...");
  _current_wait_vehicle_status.activate(
    [this](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) {return !_is_armed;},
    on_completed);
}

void ModeExecutorBase::vehicleStatusUpdated(const px4_msgs::msg::VehicleStatus::UniquePtr & msg)
{
  // Update state
  const bool was_armed = _is_armed;
  const ModeBase::ModeID current_mode = static_cast<ModeBase::ModeID>(msg->nav_state);
  _is_armed = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  const bool wants_to_activate_immediately =
    (_settings.activation == Settings::Activation::ActivateImmediately) && _was_never_activated;
  const bool is_in_charge = id() == msg->executor_in_charge &&
    (_is_armed || wants_to_activate_immediately ||
    _settings.activation == Settings::Activation::ActivateAlways);
  const bool changed_to_armed = !was_armed && _is_armed;

  bool got_activated = false;
  if (_is_in_charge != is_in_charge) {
    if (is_in_charge) {
      got_activated = true;
      callOnActivate();

    } else {
      callOnDeactivate(
        msg->failsafe ? DeactivateReason::FailsafeActivated :
        DeactivateReason::Other);
    }
  }

  if (!got_activated && _is_in_charge && _current_scheduled_mode.active() &&
    _current_scheduled_mode.modeId() == _prev_nav_state &&
    current_mode == _owned_mode.id() && _prev_nav_state != current_mode && _is_armed)
  {
    // If the user switched from the currently scheduled mode to the owned mode, the executor stays in charge.
    // In order for the executor to restore the correct state, we re-activate it (which also cancels the scheduled
    // mode)
    callOnDeactivate(DeactivateReason::Other);
    callOnActivate();
  }


  if (_is_in_charge && _prev_failsafe_defer_state != msg->failsafe_defer_state &&
    msg->failsafe_defer_state == px4_msgs::msg::VehicleStatus::FAILSAFE_DEFER_STATE_WOULD_FAILSAFE)
  {
    // FMU wants to failsafe, but got deferred -> notify the executor
    onFailsafeDeferred();
  }

  _prev_failsafe_defer_state = msg->failsafe_defer_state;

  // Do not activate the mode if we're scheduling another mode. This is only expected to happen for a brief moment,
  // e.g. when the executor gets activated or right after arming. It thus prevents unnecessary mode activation toggling.
  const bool do_not_activate_mode =
    (_current_scheduled_mode.active() && _owned_mode.id() != _current_scheduled_mode.modeId()) ||
    (changed_to_armed && _current_wait_vehicle_status.active());
  // To avoid race conditions and ensure consistent ordering we update vehicle status of the mode directly.
  _owned_mode.vehicleStatusUpdated(msg, do_not_activate_mode && _is_in_charge);

  _prev_nav_state = current_mode;

  _current_wait_vehicle_status.update(msg);
}

bool ModeExecutorBase::deferFailsafesSync(bool enabled, int timeout_s)
{
  _config_overrides.deferFailsafes(enabled, timeout_s);

  // To avoid race conditions we wait until the FMU sets it if the executor is in charge
  if (enabled && _is_in_charge && _registration->registered() &&
    _prev_failsafe_defer_state == px4_msgs::msg::VehicleStatus::FAILSAFE_DEFER_STATE_DISABLED)
  {
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(_vehicle_status_sub);

    bool got_message = false;
    auto start_time = _node.now();
    const rclcpp::Duration timeout = 1s;

    while (!got_message) {
      auto now = _node.now();

      if (now >= start_time + timeout) {
        break;
      }

      auto wait_ret = wait_set.wait(
        (timeout - (now - start_time)).to_chrono<std::chrono::microseconds>());

      if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
        px4_msgs::msg::VehicleStatus msg;
        rclcpp::MessageInfo info;

        if (_vehicle_status_sub->take(msg, info)) {
          if (msg.failsafe_defer_state !=
            px4_msgs::msg::VehicleStatus::FAILSAFE_DEFER_STATE_DISABLED)
          {
            got_message = true;
          }

        } else {
          RCLCPP_DEBUG(_node.get_logger(), "no VehicleStatus message received");
        }

      } else {
        RCLCPP_DEBUG(_node.get_logger(), "timeout");
      }
    }

    wait_set.remove_subscription(_vehicle_status_sub);

    return got_message;
  }

  return true;
}

ModeExecutorBase::ScheduledMode::ScheduledMode(
  rclcpp::Node & node,
  const std::string & topic_namespace_prefix)
{
  _mode_completed_sub = node.create_subscription<px4_msgs::msg::ModeCompleted>(
    topic_namespace_prefix + "fmu/out/mode_completed" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::ModeCompleted>(), rclcpp::QoS(
      1).best_effort(),
    [this, &node](px4_msgs::msg::ModeCompleted::UniquePtr msg) {
      if (active() && msg->nav_state == static_cast<uint8_t>(_mode_id)) {
        RCLCPP_DEBUG(
          node.get_logger(), "Got matching ModeCompleted message, result: %i",
          msg->result);
        const CompletedCallback on_completed_callback(std::move(_on_completed_callback));
        reset();
        on_completed_callback(static_cast<Result>(msg->result));                 // Call after, as it might trigger new requests
      }
    });
}

void ModeExecutorBase::ScheduledMode::activate(
  ModeBase::ModeID mode_id,
  const CompletedCallback & on_completed)
{
  assert(!active());
  _mode_id = mode_id;
  _on_completed_callback = on_completed;
}

void ModeExecutorBase::ScheduledMode::cancel()
{
  if (active()) {
    const CompletedCallback on_completed_callback(std::move(_on_completed_callback));
    reset();
    on_completed_callback(Result::Deactivated);             // Call after, as it might trigger new requests
  }
}

void ModeExecutorBase::WaitForVehicleStatusCondition::update(
  const px4_msgs::msg::VehicleStatus::UniquePtr & msg)
{
  if (_on_completed_callback && _run_check_callback(msg)) {
    const CompletedCallback on_completed_callback(std::move(_on_completed_callback));
    _on_completed_callback = nullptr;
    _run_check_callback = nullptr;
    on_completed_callback(Result::Success);             // Call after, as it might trigger new requests
  }
}

void ModeExecutorBase::WaitForVehicleStatusCondition::activate(
  const RunCheckCallback & run_check_callback,
  const CompletedCallback & on_completed)
{
  assert(!_on_completed_callback);
  _on_completed_callback = on_completed;
  _run_check_callback = run_check_callback;
}

void ModeExecutorBase::WaitForVehicleStatusCondition::cancel()
{
  if (_on_completed_callback) {
    const CompletedCallback on_completed_callback(std::move(_on_completed_callback));
    _on_completed_callback = nullptr;
    _run_check_callback = nullptr;
    on_completed_callback(Result::Deactivated);             // Call after, as it might trigger new requests
  }
}

} // namespace px4_ros2
