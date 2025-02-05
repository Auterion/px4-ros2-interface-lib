/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/mode_completed.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include "health_and_arming_checks.hpp"
#include "overrides.hpp"
#include "manual_control_input.hpp"
#include <px4_ros2/common/setpoint_base.hpp>
#include <px4_ros2/common/context.hpp>

class Registration;
struct RegistrationSettings;

namespace px4_ros2
{
/** \ingroup components
 *  @{
 */

enum class Result
{
  Success = 0,
  Rejected,       ///< The request was rejected
  Interrupted,       ///< Ctrl-C or another error (from ROS)
  Timeout,
  Deactivated,       ///< Mode or executor got deactivated

  // Mode-specific results
  ModeFailureOther = 100,
};

static_assert(
  static_cast<int>(Result::ModeFailureOther) ==
  static_cast<int>(px4_msgs::msg::ModeCompleted::RESULT_FAILURE_OTHER),
  "definition mismatch");

constexpr inline const char * resultToString(Result result) noexcept
{
  switch (result) {
    case Result::Success: return "Success";

    case Result::Rejected: return "Rejected";

    case Result::Interrupted: return "Interrupted";

    case Result::Timeout: return "Timeout";

    case Result::Deactivated: return "Deactivated";

    case Result::ModeFailureOther: return "Mode Failure (generic)";
  }

  return "Unknown";
}

/**
 * @brief Base class for a mode
 * \ingroup components
 */
class ModeBase : public Context
{
public:
  using ModeID = uint8_t;       ///< Mode ID, corresponds to nav_state
  static constexpr ModeID kModeIDInvalid = 0xff;

  static constexpr ModeID kModeIDPosctl =
    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL;
  static constexpr ModeID kModeIDTakeoff =
    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF;
  static constexpr ModeID kModeIDDescend =
    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND;
  static constexpr ModeID kModeIDLand =
    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
  static constexpr ModeID kModeIDRtl =
    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL;
  static constexpr ModeID kModeIDPrecisionLand =
    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND;
  static constexpr ModeID kModeIDLoiter =
    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER;

  struct Settings
  {
    // NOLINTNEXTLINE allow implicit conversion
    Settings(
      std::string mode_name, bool want_activate_even_while_disarmed = false,
      ModeID request_replace_internal_mode = kModeIDInvalid)
    : name(std::move(mode_name)), activate_even_while_disarmed(want_activate_even_while_disarmed),
      replace_internal_mode(request_replace_internal_mode) {}
    std::string name;             ///< Name of the mode with length < 25 characters
    bool activate_even_while_disarmed{true};             ///< If true, the mode is also activated while disarmed if selected
    ModeID replace_internal_mode{kModeIDInvalid};             ///< Can be used to replace an fmu-internal mode
  };

  ModeBase(
    rclcpp::Node & node, Settings settings,
    const std::string & topic_namespace_prefix = "");
  ModeBase(const ModeBase &) = delete;
  virtual ~ModeBase() = default;

  /**
   * Register the mode. Call this once on startup, unless there's an associated executor. This is a blocking method.
   * @return true on success
   */
  bool doRegister();


  /**
   * Report any custom mode requirements. This is called regularly, also while the mode is active.
   */
  virtual void checkArmingAndRunConditions(HealthAndArmingCheckReporter & reporter) {}

  /**
   * Called whenever the mode is activated, also if the vehicle is disarmed
   */
  virtual void onActivate() = 0;

  /**
   * Called whenever the mode is deactivated, also if the vehicle is disarmed
   */
  virtual void onDeactivate() = 0;

  /**
   * Set the update rate when the mode is active. This is set automatically from the configured setpoints, but can
   * be set as needed.
   * @param rate_hz set to 0 to disable
   */
  void setSetpointUpdateRate(float rate_hz);

  virtual void updateSetpoint(float dt_s) {}

  /**
   * Mode completed signal. Call this when the mode is finished. A mode might never call this, but modes like
   * RTL, Land or Takeoff are expected to signal their completion.
   * @param result
   */
  void completed(Result result);


  // Properties & state

  ModeID id() const;

  bool isArmed() const {return _is_armed;}

  bool isActive() const {return _is_active;}

  ConfigOverrides & configOverrides() {return _config_overrides;}

  /**
   * Get / modify mode requirements. These are generally automatically set based on selected setpoint types,
   * and are used to prevent arming or trigger failsafes.
   */
  RequirementFlags & modeRequirements() {return _health_and_arming_checks.modeRequirements();}

protected:
  void setSkipMessageCompatibilityCheck() {_skip_message_compatibility_check = true;}
  void overrideRegistration(const std::shared_ptr<Registration> & registration);

private:
  void addSetpointType(SetpointBase * setpoint) override;
  void setRequirement(const RequirementFlags & requirement_flags) override;

  friend class ModeExecutorBase;
  RegistrationSettings getRegistrationSettings() const;
  void onAboutToRegister();
  bool onRegistered();

  void unsubscribeVehicleStatus();
  void vehicleStatusUpdated(
    const px4_msgs::msg::VehicleStatus::UniquePtr & msg,
    bool do_not_activate = false);

  void callOnActivate();
  void callOnDeactivate();

  void updateSetpointUpdateTimer();

  void updateModeRequirementsFromSetpoints();
  void setSetpointUpdateRateFromSetpointTypes();
  void activateSetpointType(SetpointBase & setpoint);

  std::shared_ptr<Registration> _registration;

  const Settings _settings;
  bool _skip_message_compatibility_check{false};

  HealthAndArmingChecks _health_and_arming_checks;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
  rclcpp::Publisher<px4_msgs::msg::ModeCompleted>::SharedPtr _mode_completed_pub;
  rclcpp::Publisher<px4_msgs::msg::VehicleControlMode>::SharedPtr _config_control_setpoints_pub;

  bool _is_active{false};       ///< Mode is currently selected
  bool _is_armed{false};       ///< Is vehicle armed?
  bool _completed{false};       ///< Is mode completed?

  float _setpoint_update_rate_hz{0.f};
  rclcpp::TimerBase::SharedPtr _setpoint_update_timer;
  rclcpp::Time _last_setpoint_update{};

  ConfigOverrides _config_overrides;

  std::vector<std::shared_ptr<SetpointBase>> _setpoint_types;
  std::vector<SetpointBase *> _new_setpoint_types; ///< This stores new setpoints during initialization, until registration
};

/** @}*/
} // namespace px4_ros2
