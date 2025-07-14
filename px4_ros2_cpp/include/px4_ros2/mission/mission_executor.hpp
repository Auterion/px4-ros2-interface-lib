/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once
#include <memory>
#include <vector>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/mission/actions/action.hpp>
#include <px4_ros2/mission/trajectory/trajectory_executor.hpp>
#include <px4_ros2/vehicle_state/land_detected.hpp>

namespace px4_ros2
{
/** \ingroup mission
 *  @{
 */

class AsyncFunctionCalls;
class ActionStateKeeper;

class MissionExecutor
{
public:
  struct Configuration
  {
    std::vector<std::function<std::shared_ptr<ActionInterface>(ModeBase & mode)>>
    custom_actions_factory;
    std::set<std::string> default_actions{"changeSettings", "onResume", "onFailure", "takeoff",
      "rtl", "land", "hold"};
    std::function<std::shared_ptr<TrajectoryExecutorInterface>(ModeBase & mode)>
    trajectory_executor_factory;
    std::string persistence_filename;

    template<class ActionType, typename ... Args>
    Configuration & addCustomAction(Args &&... args)
    {
      custom_actions_factory.push_back(
        [args = std::tie(std::forward<Args>(args) ...)](ModeBase & mode)mutable {
          return std::apply(
            [&mode](auto &&... args)
            {
              return std::make_shared<ActionType>(mode, std::forward<Args>(args)...);
            }, std::move(args));
        });
      return *this;
    }
    template<typename TrajectoryExecutorType, typename ... Args>
    Configuration & withTrajectoryExecutor(Args &&... args)
    {
      trajectory_executor_factory =
        [args = std::tie(std::forward<Args>(args) ...)](ModeBase & mode)mutable {
          return std::apply(
            [&mode](auto &&... args)
            {
              return std::make_shared<TrajectoryExecutorType>(mode, std::forward<Args>(args)...);
            }, std::move(args));
        };
      return *this;
    }
    Configuration & withPersistenceFile(const std::string & filename)
    {
      persistence_filename = filename;
      return *this;
    }
  };

  MissionExecutor(
    const std::string & mode_name, const Configuration & configuration,
    rclcpp::Node & node, const std::string & topic_namespace_prefix = "");

  virtual ~MissionExecutor();

  bool doRegister();

  void setMission(const Mission & mission);

  const Mission & mission() const {return *_mission;}

  void onActivated(const std::function<void()> & callback) {_on_activated = callback;}
  void onDeactivated(const std::function<void()> & callback) {_on_deactivated = callback;}
  /**
   * Get notified when the current mission item changes. This is the index into the current mission items
   * that is currently being executed (i.e. if it is a waypoint, the current target waypoint)
   */
  void onProgressUpdate(const std::function<void(int)> & callback) {_on_progress_update = callback;}
  void onCompleted(const std::function<void()> & callback) {_on_completed = callback;}
  void onFailsafeDeferred(const std::function<void()> & callback);
  void onReadynessUpdate(
    const std::function<void(bool ready,
    const std::vector<std::string> & errors)> & callback);

  /**
   * Enable/disable deferring failsafes. While enabled (and the mission executor is active),
   * most failsafes are prevented from being triggered until the given timeout is exceeded.
   * Some failsafes that cannot be prevented:
   * - vehicle exceeds attitude limits (can be disabled via parameters)
   * - the mode cannot run (some mode requirements are not met, such as no position estimate)
   * @param enabled
   * @param timeout_s 0=system default, -1=no timeout
   * @return true on success
   */
  bool deferFailsafes(bool enabled, int timeout_s = 0);

  bool controlAutoSetHome(bool enabled);

  /**
   * Abort a currently running mission. This will trigger the onFailure action (which can be customized).
   * The abort reason will be set to 'other'.
   */
  void abort();

protected:
  class MissionMode : public ModeBase
  {
public:
    MissionMode(
      rclcpp::Node & node, const Settings & settings, const std::string & topic_namespace_prefix,
      MissionExecutor & mission_executor)
    : ModeBase(node, settings, topic_namespace_prefix), _mission_executor(mission_executor)
    {
    }

    explicit MissionMode(const ModeBase & mode_base) = delete;

    ~MissionMode() override = default;
    void checkArmingAndRunConditions(HealthAndArmingCheckReporter & reporter) override
    {
      _mission_executor.checkArmingAndRunConditions(reporter);
    }
    void onActivate() override {}
    void onDeactivate() override {}
    void updateSetpoint(float dt_s) override {_mission_executor.updateSetpoint();}

    void disableWatchdogTimer() // NOLINT we just want to change the methods visibility
    {
      ModeBase::disableWatchdogTimer();
    }

private:
    MissionExecutor & _mission_executor;
  };

  class MissionModeExecutor : public ModeExecutorBase
  {
public:
    MissionModeExecutor(
      rclcpp::Node & node, const Settings & settings, ModeBase & owned_mode,
      const std::string & topic_namespace_prefix, MissionExecutor & mission_executor)
    : ModeExecutorBase(node, settings, owned_mode, topic_namespace_prefix), _mission_executor(
        mission_executor)
    {
    }

    explicit MissionModeExecutor(const ModeExecutorBase & mode_executor_base) = delete;

    ~MissionModeExecutor() override = default;
    void onActivate() override {_mission_executor.onActivate();}
    void onDeactivate(DeactivateReason reason) override {_mission_executor.onDeactivate(reason);}
    void onFailsafeDeferred() override
    {
      if (on_failsafe_deferred) {
        on_failsafe_deferred();
      }
    }

    Result sendCommandSync(
      uint32_t command, float param1, float param2, float param3,
      float param4, float param5, float param6, float param7) override
    {
      if (command_handler) {
        return command_handler(
          command,
          param1) ? Result::Success : Result::Rejected;
      }
      return ModeExecutorBase::sendCommandSync(
        command, param1, param2, param3, param4, param5,
        param6, param7);
    }

    void setRegistration(const std::shared_ptr<Registration> & registration)
    {
      setSkipMessageCompatibilityCheck();
      overrideRegistration(registration);
    }
    std::function<bool(uint32_t, float)> command_handler{nullptr};
    std::function<void()> on_failsafe_deferred{nullptr};

private:
    MissionExecutor & _mission_executor;
  };

  virtual bool doRegisterImpl(MissionMode & mode, MissionModeExecutor & executor_base);

  void setCommandHandler(const std::function<bool(uint32_t, float)> & command_handler)
  {
    _mode_executor->command_handler = command_handler;
  }

  std::shared_ptr<LandDetected> _land_detected;

private:
  using ActionID = int;
  enum class AbortReason
  {
    Other,
    ModeFailure,
    TrajectoryFailure,
    NoValidMission,
    ActionDoesNotExist,
  };
  static std::string abortReasonStr(AbortReason reason);

  void checkArmingAndRunConditions(HealthAndArmingCheckReporter & reporter);
  void checkReadynessAndReport();
  void updateSetpoint();
  void onActivate();
  void onDeactivate(ModeExecutorBase::DeactivateReason reason);

  bool isReady() const {return _has_valid_mission && _actions_ready;}

  void runMode(
    ModeBase::ModeID mode_id,
    const std::function<void()> & on_completed, const std::function<void()> & on_failure = nullptr);
  void runModeTakeoff(float altitude, float heading,
    const std::function<void()> & on_completed, const std::function<void()> & on_failure = nullptr);
  void runAction(
    const std::string & action_name, const ActionArguments & arguments,
    const std::function<void()> & on_completed);
  void runTrajectory(
    const std::shared_ptr<Mission> & trajectory, int start_index, int end_index,
    const std::function<void(int)> & on_index_reached, bool stop_at_last_item);

  void runNextMissionItem();
  void runCurrentMissionItem(bool resuming);
  void resetMissionState();
  std::pair<int, bool> getNextTrajectorySegment(int start_index) const;
  bool currentActionSupportsResumeFromLanded() const;

  void setTrajectoryOptions(const TrajectoryOptions & options);
  void clearTrajectoryOptions();

  void setCurrentMissionIndex(int index);

  void abort(AbortReason reason);
  void invalidateActionHandler();

  void savePersistentState();
  void clearPersistentState() const;
  bool tryLoadPersistentState();
  nlohmann::json getOnResumeStateAndClear();
  void runOnResumeStoreState();

  struct ActionState
  {
    std::string name;
    ActionArguments arguments;
  };
  std::unique_ptr<ActionStateKeeper> addContinousAction(const ActionState & state);
  void removeContinousAction(ActionID id);
  void runStoredActions();
  void deactivateAllActions();

  struct PersistentState
  {
    std::optional<int> current_index;
    std::string mission_checksum;

    std::map<ActionID, ActionState> continuous_actions;

    void toJson(nlohmann::json & j) const;
    void fromJson(const nlohmann::json & j);
  };

  PersistentState _state;
  int _next_continuous_action_id{0};
  const std::string _persistence_filename;

  enum class MissionItemState
  {
    Trajectory,     ///< Currently running a trajectory
    Other,
  };
  MissionItemState _mission_item_state{MissionItemState::Other};
  ModeBase::ModeID _active_mode{};
  bool _is_active{false};

  std::optional<rclcpp::Time> _trajectory_update_warn;

  std::shared_ptr<TrajectoryExecutorInterface> _trajectory_executor;
  std::map<std::string, std::shared_ptr<ActionInterface>> _actions;
  std::shared_ptr<Mission> _mission;
  TrajectoryOptions _trajectory_options; ///< current trajectory options
  bool _has_valid_mission{false};
  std::vector<std::string> _mission_errors{{"No mission"}};
  bool _actions_ready{false};
  rclcpp::TimerBase::SharedPtr _readyness_timer;
  int _abort_recursion_level{0}; ///< abort() might be called recursively, this stores the current level

  rclcpp::Node & _node;
  std::unique_ptr<MissionMode> _mode;
  std::unique_ptr<MissionModeExecutor> _mode_executor;
  std::shared_ptr<ActionHandler> _action_handler;

  std::unique_ptr<AsyncFunctionCalls> _reporting; // Report asynchronously to avoid potential recursive calls and state changes in between
  std::function<void()> _on_activated;
  std::function<void()> _on_deactivated;
  std::function<void(int)> _on_progress_update;
  std::function<void()> _on_completed;
  std::function<void(bool ready, const std::vector<std::string> & errors)> _on_readyness_update;

  friend class ActionHandler;
  friend class ActionStateKeeper;
};

class ActionStateKeeper
{
public:
  ActionStateKeeper(int id, MissionExecutor & mission_executor)
  : _id(id), _mission_executor(mission_executor) {}

  ~ActionStateKeeper()
  {
    _mission_executor.removeContinousAction(_id);
  }

private:
  const int _id;
  MissionExecutor & _mission_executor;
};

class ActionHandler
{
public:
  explicit ActionHandler(MissionExecutor & mission_executor)
  : _mission_executor(mission_executor) {}

  void runMode(
    ModeBase::ModeID mode_id,
    const std::function<void()> & on_completed, const std::function<void()> & on_failure = nullptr)
  {
    if (!_valid) {
      RCLCPP_WARN(_mission_executor._node.get_logger(), "ActionHandler is not valid anymore");
      return;
    }
    _mission_executor.runMode(mode_id, on_completed, on_failure);
  }
  void runModeTakeoff(float altitude, float heading,
    const std::function<void()> & on_completed, const std::function<void()> & on_failure = nullptr)
  {
    if (!_valid) {
      RCLCPP_WARN(_mission_executor._node.get_logger(), "ActionHandler is not valid anymore");
      return;
    }
    _mission_executor.runModeTakeoff(altitude, heading, on_completed, on_failure);
  }
  void runAction(
    const std::string & action_name, const ActionArguments & arguments,
    const std::function<void()> & on_completed)
  {
    if (!_valid) {
      RCLCPP_WARN(_mission_executor._node.get_logger(), "ActionHandler is not valid anymore");
      return;
    }
    _mission_executor.runAction(action_name, arguments, on_completed);
  }
  void runTrajectory(
    const std::shared_ptr<Mission> & trajectory,
    const std::function<void()> & on_completed, bool stop_at_last_item = true);

  /**
   * @brief get the current trajectory config options
   */
  TrajectoryOptions getTrajectoryOptions() const {return _mission_executor._trajectory_options;}
  /**
   * @brief Reset the current trajectory config options to the mission defaults
   */
  void clearTrajectoryOptions()
  {
    if (!_valid) {
      RCLCPP_WARN(_mission_executor._node.get_logger(), "ActionHandler is not valid anymore");
      return;
    }
    _mission_executor.clearTrajectoryOptions();
  }
  /**
   * @brief Override the trajectory config options
   *
   * The values that are not set will be kept as they were before
   */
  void setTrajectoryOptions(const TrajectoryOptions & options)
  {
    if (!_valid) {
      RCLCPP_WARN(_mission_executor._node.get_logger(), "ActionHandler is not valid anymore");
      return;
    }
    _mission_executor.setTrajectoryOptions(options);
  }


  std::optional<int> getCurrentMissionIndex() const;

  bool currentActionSupportsResumeFromLanded() const;

  /**
   * @brief store the state of a continuous action
   *
   * This is used to restore the state when the mission is interrupted (e.g. by the user or a failsafe) and
   * continued later on. When restoring, the action will be called with the provided arguments.
   *
   * This can be used for example, for a camera trigger command that continuous to run in the background.
   * @param action_name Name of the calling action
   * @param arguments Arguments to the action for restoring the action
   * @return A token that should be stored. It can be deleted when the action stops or gets deactivated
   */
  std::unique_ptr<ActionStateKeeper> storeState(
    const std::string & action_name,
    const ActionArguments & arguments)
  {
    if (!_valid) {
      return nullptr;
    }
    return _mission_executor.addContinousAction(
      MissionExecutor::ActionState{action_name,
        arguments});
  }

  const Mission & mission() const {return _mission_executor.mission();}

  /**
   * @brief enable/disable deferral of failsafes
   *
   * @param enabled
   * @param timeout_s 0=system default, -1=no timeout
   * @return true on success
   */
  bool deferFailsafes(bool enabled, int timeout_s = 0)
  {
    if (!_valid) {
      RCLCPP_WARN(_mission_executor._node.get_logger(), "ActionHandler is not valid anymore");
      return false;
    }
    return _mission_executor.deferFailsafes(enabled, timeout_s);
  }

  bool controlAutoSetHome(bool enabled)
  {
    if (!_valid) {
      RCLCPP_WARN(_mission_executor._node.get_logger(), "ActionHandler is not valid anymore");
      return false;
    }
    return _mission_executor.controlAutoSetHome(enabled);
  }

  /**
   * @brief register callback for failsafe notification
   *
   * The callback is triggerd when a failsafe occurs while deferral is enabled.
   */
  void onFailsafeDeferred(const std::function<void()> & callback)
  {
    if (!_valid) {
      RCLCPP_WARN(_mission_executor._node.get_logger(), "ActionHandler is not valid anymore");
      return;
    }
    _mission_executor.onFailsafeDeferred(callback);
  }

  /**
   * @brief Check if the handler is still valid
   *
   * If the handler is invalid, it cannot be used anymore. It will be set invalid when the mission is aborted,
   * or deactivated.
   * This is useful for actions that use timers to run actions.
   */
  bool isValid() const {return _valid;}

private:
  friend class MissionExecutor;
  void setInvalid() {_valid = false;}

  bool _valid{true};
  MissionExecutor & _mission_executor;
};

/** @}*/
} /* namespace px4_ros2 */
