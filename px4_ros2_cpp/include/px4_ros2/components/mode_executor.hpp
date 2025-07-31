/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include "mode.hpp"
#include "overrides.hpp"

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/mode_completed.hpp>

#include <functional>

class Registration;

namespace px4_ros2
{
/** \ingroup components
 *  @{
 */

/**
 * @brief Base class for a mode executor
 */
class ModeExecutorBase
{
public:
  using CompletedCallback = std::function<void (Result)>;

  struct Settings
  {
    enum class Activation
    {
      ActivateOnlyWhenArmed, ///< Only activate the executor when armed (and selected)
      ActivateAlways, ///< Allow the executor to always be activated (so it can arm the vehicle)
      ActivateImmediately, ///< Activate the mode and executor immediately after registration. Only use this for fully autonomous executors that also arm the vehicle
    };
    Activation activation{Activation::ActivateOnlyWhenArmed};
  };

  enum class DeactivateReason
  {
    FailsafeActivated,
    Other
  };

  ModeExecutorBase(
    rclcpp::Node & node, const Settings & settings, ModeBase & owned_mode,
    const std::string & topic_namespace_prefix = "");
  ModeExecutorBase(const ModeExecutorBase &) = delete;
  virtual ~ModeExecutorBase() = default;

  /**
   * Register the mode executor. Call this once on startup. This is a blocking method.
   * @return true on success
   */
  bool doRegister();


  /**
   * Called whenever the mode is activated, also if the vehicle is disarmed
   */
  virtual void onActivate() = 0;

  /**
   * Called whenever the mode is deactivated, also if the vehicle is disarmed
   */
  virtual void onDeactivate(DeactivateReason reason) = 0;

  /**
   * Called when failsafes are currently being deferred, and the FMU wants to trigger a failsafe.
   * @see deferFailsafesSync()
   */
  virtual void onFailsafeDeferred() {}

  /**
  * Send command and wait for ack/nack
  */
  Result sendCommandSync(
    uint32_t command, float param1 = NAN, float param2 = NAN, float param3 = NAN,
    float param4 = NAN,
    float param5 = NAN, float param6 = NAN, float param7 = NAN);

  /**
   * Switch to a mode with a callback when it is finished.
   * The callback is also executed when the mode is deactivated.
   * If there's already a mode scheduling active, the previous one is cancelled.
   *
   * The forced parameter, when set to true, allows to be able to force the scheduling of the modes when the drone is disarmed
   */
  void scheduleMode(
    ModeBase::ModeID mode_id, const CompletedCallback & on_completed,
    bool forced = false);

  void takeoff(const CompletedCallback & on_completed, float altitude = NAN, float heading = NAN);
  void land(const CompletedCallback & on_completed);
  void rtl(const CompletedCallback & on_completed);

  void arm(const CompletedCallback & on_completed, bool run_preflight_checks = true);
  void disarm(const CompletedCallback & on_completed, bool forced = false);
  void waitReadyToArm(const CompletedCallback & on_completed);
  void waitUntilDisarmed(const CompletedCallback & on_completed);

  bool isInCharge() const {return _is_in_charge;}

  bool isArmed() const {return _is_armed;}

  ModeBase & ownedMode() {return _owned_mode;}

  int id() const;

  rclcpp::Node & node() {return _node;}

  ConfigOverrides & configOverrides() {return _config_overrides;}

  /**
   * Enable/disable deferring failsafes. While enabled (and the executor is in charge),
   * most failsafes are prevented from being triggered until the given timeout is exceeded.
   * Some failsafes that cannot be prevented:
   * - vehicle exceeds attitude limits (can be disabled via parameters)
   * - the mode cannot run (some mode requirements are not met, such as no position estimate)
   *
   * If the executor is in charge, this method will wait for the FMU for acknowledgement (to avoid race conditions)
   * @param enabled
   * @param timeout_s 0=system default, -1=no timeout
   * @return true on success
   */
  bool deferFailsafesSync(bool enabled, int timeout_s = 0);

private:
  class ScheduledMode
  {
public:
    ScheduledMode(rclcpp::Node & node, const std::string & topic_namespace_prefix);

    bool active() const {return _mode_id != ModeBase::kModeIDInvalid;}
    void activate(ModeBase::ModeID mode_id, const CompletedCallback & on_completed);
    void cancel();
    ModeBase::ModeID modeId() const {return _mode_id;}

private:
    void reset() {_mode_id = ModeBase::kModeIDInvalid;}

    ModeBase::ModeID _mode_id{ModeBase::kModeIDInvalid};
    CompletedCallback _on_completed_callback;
    rclcpp::Subscription<px4_msgs::msg::ModeCompleted>::SharedPtr _mode_completed_sub;
  };

  class WaitForVehicleStatusCondition
  {
public:
    using RunCheckCallback =
      std::function<bool (const px4_msgs::msg::VehicleStatus::UniquePtr & msg)>;
    WaitForVehicleStatusCondition() = default;

    bool active() const {return _on_completed_callback != nullptr;}
    void update(const px4_msgs::msg::VehicleStatus::UniquePtr & msg);

    void activate(
      const RunCheckCallback & run_check_callback,
      const CompletedCallback & on_completed);
    void cancel();

private:
    CompletedCallback _on_completed_callback;
    RunCheckCallback _run_check_callback;
  };

  void onRegistered();

  void callOnActivate();
  void callOnDeactivate(DeactivateReason reason);

  void vehicleStatusUpdated(const px4_msgs::msg::VehicleStatus::UniquePtr & msg);

  void scheduleMode(
    ModeBase::ModeID mode_id, const px4_msgs::msg::VehicleCommand & cmd,
    const ModeExecutorBase::CompletedCallback & on_completed, bool forced = false);

  rclcpp::Node & _node;
  const std::string _topic_namespace_prefix;
  const Settings _settings;
  ModeBase & _owned_mode;

  std::shared_ptr<Registration> _registration;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;

  ScheduledMode _current_scheduled_mode;
  WaitForVehicleStatusCondition _current_wait_vehicle_status;

  bool _is_in_charge{false};
  bool _is_armed{false};
  bool _was_never_activated{true};
  ModeBase::ModeID _prev_nav_state{ModeBase::kModeIDInvalid};
  uint8_t _prev_failsafe_defer_state{px4_msgs::msg::VehicleStatus::FAILSAFE_DEFER_STATE_DISABLED};

  ConfigOverrides _config_overrides;
};

/** @}*/
} // namespace px4_ros2
