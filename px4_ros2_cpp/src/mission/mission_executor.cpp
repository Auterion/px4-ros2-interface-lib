/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <fstream>
#include <queue>
#include <filesystem>
#include <fcntl.h>
#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/mission/actions/action.hpp>
#include <px4_ros2/mission/actions/default_actions.hpp>
#include <px4_ros2/mission/trajectory/multicopter/waypoint_trajectory_executor.hpp>
#include <px4_ros2/utils/visit.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>

namespace px4_ros2
{

class AsyncFunctionCalls
{
public:
  explicit AsyncFunctionCalls(rclcpp::Node & node)
  : _node(node)
  {
    // Use a timer for deferred callbacks, which is the simplest solution
    _update_timer = _node.create_wall_timer(
      1ms, [this]
      {
        _update_timer->cancel();
        while (!_pending.empty()) {
          const auto & cb = _pending.front();
          cb();
          _pending.pop();
        }
      });
    _update_timer->cancel();
  }

  void callAsync(const std::function<void()> & callback)
  {
    _pending.push(callback);
    _update_timer->reset();
  }

private:
  std::queue<std::function<void()>> _pending;
  rclcpp::TimerBase::SharedPtr _update_timer;
  rclcpp::Node & _node;
};


MissionExecutor::MissionExecutor(
  const std::string & mode_name, const Configuration & configuration, rclcpp::Node & node,
  const std::string & topic_namespace_prefix)
: _persistence_filename(configuration.persistence_filename), _node(node),
  _reporting(std::make_unique<AsyncFunctionCalls>(
      node))
{
  _mode = std::make_unique<MissionMode>(node, mode_name, topic_namespace_prefix, *this);
  _mode_executor = std::make_unique<MissionModeExecutor>(
    node, ModeExecutorBase::Settings{},
    *_mode, topic_namespace_prefix, *this);

  _land_detected = std::make_shared<LandDetected>(*_mode);

  // Custom actions
  for (const auto & action_factory : configuration.custom_actions_factory) {
    const std::shared_ptr<ActionInterface> action = action_factory(*_mode);
    if (_actions.find(action->name()) != _actions.end()) {
      RCLCPP_ERROR(_node.get_logger(), "Duplicate action '%s'", action->name().c_str());
      continue;
    }
    _actions[action->name()] = action;
  }
  // Default actions
  for (const auto & default_action : configuration.default_actions) {
    if (_actions.find(default_action) != _actions.end()) {
      continue;
    }
    if (default_action == "rtl") {
      _actions[default_action] = std::make_shared<default_actions::Rtl>();
    } else if (default_action == "land") {
      _actions[default_action] = std::make_shared<default_actions::Land>();
    } else if (default_action == "takeoff") {
      _actions[default_action] = std::make_shared<default_actions::Takeoff>();
    } else if (default_action == "onFailure") {
      _actions[default_action] = std::make_shared<default_actions::OnFailure>();
    } else if (default_action == "onResume") {
      _actions[default_action] =
        std::make_shared<default_actions::OnResume>(*_mode, _land_detected);
    } else if (default_action == "hold") {
      _actions[default_action] = std::make_shared<default_actions::Hold>(*_mode);
    } else if (default_action == "changeSettings") {
      _actions[default_action] = std::make_shared<default_actions::ChangeSettings>();

    } else {
      RCLCPP_ERROR(_node.get_logger(), "Unknown default action '%s'", default_action.c_str());
    }
  }


  // Trajectory execution
  if (configuration.trajectory_executor_factory) {
    _trajectory_executor = configuration.trajectory_executor_factory(*_mode);
  } else {
    // TODO: vehicle-type specific trajectory executor
    _trajectory_executor = std::make_shared<multicopter::WaypointTrajectoryExecutor>(*_mode);
  }

  // Make sure there's always a valid mission object
  _mission = std::make_shared<Mission>();

  _action_handler = std::make_shared<ActionHandler>(*this);

  _readyness_timer = _node.create_wall_timer(
    1s, [this] {
      checkReadynessAndReport();
    });
}

MissionExecutor::~MissionExecutor() // NOLINT destructor needs to be in .cpp file for forward declaration
{
}

void MissionExecutor::setMission(const Mission & mission)
{
  if (_is_active) {
    RCLCPP_ERROR(_node.get_logger(), "Cannot set the mission, already active");
    return;
  }

  // Check if the mission is feasible:
  // - all frame types are supported
  // - all navigation item types are supported
  // - all actions are known
  // - mission is not empty
  _mission_errors.clear();
  bool navigation_item_unsupported = false;
  bool frame_unsupported = false;
  for (const auto & mission_item : mission.items()) {
    std::visit(
      util::Overloaded{
        [&](const NavigationItem & navigation_item)
        {
          std::visit(
            util::Overloaded{
            [&](const Waypoint & waypoint)
            {
              if (!_trajectory_executor->navigationItemTypeSupported(NavigationItemType::Waypoint))
              {
                if (!navigation_item_unsupported) {
                  _mission_errors.emplace_back(
                    "Mission contains unsupported navigation item: waypoint");
                  navigation_item_unsupported = true;
                }
              }
              if (!_trajectory_executor->frameSupported(waypoint.frame)) {
                if (!frame_unsupported) {
                  _mission_errors.emplace_back(
                    "Mission contains navigation item with unsupported frame: " +
                    missionFrameStr(waypoint.frame));
                  frame_unsupported = true;
                }
              }
            }}, navigation_item.data);
        },
        [&](const ActionItem & action_item)
        {
          if (_actions.find(action_item.name) == _actions.end()) {
            _mission_errors.push_back("Mission contains unknown action '" + action_item.name + "'");
          }
        },
      }, mission_item);
  }
  if (mission.items().empty()) {
    _mission_errors.emplace_back("Mission contains no mission items");
  }

  _mission = std::make_shared<Mission>(mission);
  _state.mission_checksum = _mission->checksum();
  _has_valid_mission = _mission_errors.empty();
  checkReadynessAndReport();

  if (!tryLoadPersistentState()) {
    resetMissionState();
  }
}

void MissionExecutor::onFailsafeDeferred(const std::function<void()> & callback)
{
  _mode_executor->on_failsafe_deferred = callback;
}

void MissionExecutor::onReadynessUpdate(
  const std::function<void(bool ready, const std::vector<std::string> & errors)> & callback)
{
  _on_readyness_update = callback;
  checkReadynessAndReport();
}

bool MissionExecutor::controlAutoSetHome(bool enabled)
{
  return _mode_executor->controlAutoSetHome(enabled);
}

bool MissionExecutor::deferFailsafes(bool enabled, int timeout_s)
{
  return _mode_executor->deferFailsafesSync(enabled, timeout_s);
}

void MissionExecutor::abort()
{
  if (_is_active) {
    abort(AbortReason::Other);
  }
}

bool MissionExecutor::doRegister()
{
  return doRegisterImpl(*_mode, *_mode_executor);
}

bool MissionExecutor::tryLoadPersistentState()
{
  if (_persistence_filename.empty()) {
    return false;
  }
  PersistentState state;
  std::ifstream file(_persistence_filename);
  if (!file.is_open()) {
    return false;
  }
  bool ret = false;
  try {
    nlohmann::json j;
    file >> j;
    state.fromJson(j);
    if (state.mission_checksum == _mission->checksum() && state.current_index.has_value() &&
      *state.current_index < static_cast<int>(_mission->items().size()))
    {
      _state = state;
      RCLCPP_DEBUG(_node.get_logger(), "Restored mission state to index %i", *_state.current_index);
      ret = true;
    } else {
      RCLCPP_DEBUG(
        _node.get_logger(),
        "Stored mission state checksum mismatch or index out of bounds, discarding");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(_node.get_logger(), "Failed trying to parse mission state: %s", e.what());
  }
  return ret;
}

nlohmann::json MissionExecutor::getOnResumeStateAndClear()
{
  for (const auto & [id, action] : _state.continuous_actions) {
    if (action.name == "onResume") {
      const auto ret = action.arguments.json();
      _state.continuous_actions.erase(id);
      return ret;
    }
  }
  return {};
}

void MissionExecutor::runOnResumeStoreState()
{
  const auto iter = _actions.find("onResume");
  if (iter != _actions.end()) {
    nlohmann::json arguments;
    arguments["action"] = "storeState";
    iter->second->run(_action_handler, ActionArguments(arguments), [] {});
  }
}

std::unique_ptr<ActionStateKeeper> MissionExecutor::addContinousAction(const ActionState & state)
{
  const ActionID id = _next_continuous_action_id++;
  RCLCPP_DEBUG(
    _node.get_logger(), "Adding continuous action id %i, action name '%s'", id,
    state.name.c_str());
  _state.continuous_actions[id] = state;
  return std::make_unique<ActionStateKeeper>(id, *this);
}

void MissionExecutor::removeContinousAction(ActionID id)
{
  if (!_is_active) {
    // If the state is removed as a result of the deactivate() call, we want to keep it so it can be restored later on
    // (if not stored persistently)
    return;
  }
  auto iter = _state.continuous_actions.find(id);
  if (iter == _state.continuous_actions.end()) {
    RCLCPP_WARN(_node.get_logger(), "Stored action state with id %i not found", id);
    return;
  }
  RCLCPP_DEBUG(_node.get_logger(), "Removing continuous action id %i", id);
  _state.continuous_actions.erase(iter);
}

void MissionExecutor::runStoredActions()
{
  // Create a copy of the actions, and reset them, as they are expected to be added again after running them
  const auto actions = _state.continuous_actions;
  _state.continuous_actions = {};

  // Run all actions
  for (const auto & action : actions) {
    const auto iter = _actions.find(action.second.name);
    if (iter == _actions.end()) {
      RCLCPP_WARN(
        _node.get_logger(), "Stored action name '%s' not found",
        action.second.name.c_str());
      continue;
    }
    // onResume is applied earlier
    if (iter->second->name() == "onResume") {
      continue;
    }
    iter->second->run(_action_handler, action.second.arguments, [] {});
  }

  // We expect all restored actions to have created another continuous action, so the action counts should be equal
  if (!actions.empty()) {
    RCLCPP_DEBUG(
      _node.get_logger(), "Restored %lu actions (new stored count: %lu)",
      actions.size(), _state.continuous_actions.size());
  }
}

void MissionExecutor::deactivateAllActions()
{
  for (auto & [name, action] : _actions) {
    action->deactivate();
  }
}

void MissionExecutor::savePersistentState()
{
  if (_persistence_filename.empty()) {
    return;
  }

  if (!_state.current_index.has_value() || *_state.current_index < 0 ||
    *_state.current_index >= static_cast<int>(_mission->items().size()))
  {
    clearPersistentState();
  } else {
    RCLCPP_DEBUG(
      _node.get_logger(), "Saving persistent state to file '%s'", _persistence_filename.c_str());

    int fd = open(_persistence_filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR);
    if (fd < 0) {
      RCLCPP_ERROR(
        _node.get_logger(), "Failed to open file '%s' for writing (%s)",
        _persistence_filename.c_str(), strerror(errno));
      return;
    }
    nlohmann::json j;
    _state.toJson(j);
    const std::string json_str = j.dump();
    if (write(fd, json_str.c_str(), json_str.size()) != static_cast<ssize_t>(json_str.size())) {
      RCLCPP_ERROR(
        _node.get_logger(), "Failed to write JSON to file '%s'", _persistence_filename.c_str());
    }
    fsync(fd);
    close(fd);

    // Call fsync on the parent directory to ensure the directory entry is flushed to disk as well
    fd = open(std::filesystem::path(_persistence_filename).parent_path().c_str(), O_RDONLY);
    if (fd >= 0) {
      fsync(fd);
      close(fd);
    }
  }
}

void MissionExecutor::clearPersistentState() const
{
  if (!_persistence_filename.empty()) {
    RCLCPP_DEBUG(
      _node.get_logger(), "Removing persistent state file '%s'", _persistence_filename.c_str());
    std::remove(_persistence_filename.c_str());
    // Call fsync on the parent directory to ensure it's flushed to disk
    const int fd =
      open(std::filesystem::path(_persistence_filename).parent_path().c_str(), O_RDONLY);
    if (fd >= 0) {
      fsync(fd);
      close(fd);
    }
  }
}
void MissionExecutor::PersistentState::toJson(nlohmann::json & j) const
{
  j["version"] = 1;
  if (current_index.has_value()) {
    j["currentIndex"] = *current_index;
  }
  nlohmann::json::array_t actions;
  for (const auto & [id, action] : continuous_actions) {
    actions.push_back({{"name", action.name}, {"arguments", action.arguments.json()}});
  }
  j["actions"] = actions;
  j["checksum"] = mission_checksum;
}
void MissionExecutor::PersistentState::fromJson(const nlohmann::json & j)
{
  const int version = j.at("version").get<int>();
  if (version != 1) {
    throw nlohmann::json::other_error::create(0, "Unknown version: " + std::to_string(version), &j);
  }
  if (j.contains("currentIndex")) {
    current_index = j.at("currentIndex").get<int>();
  }
  if (j.contains("actions")) {
    int id = 0;
    for (const auto & action : j.at("actions")) {
      auto & stored_action = continuous_actions[id++];
      stored_action.name = action.at("name");
      stored_action.arguments = ActionArguments(action.at("arguments"));
    }
  }
  mission_checksum = j.at("checksum").get<std::string>();
}

bool MissionExecutor::doRegisterImpl(MissionMode & mode, MissionModeExecutor & executor_base)
{
  return executor_base.doRegister();
}

std::string MissionExecutor::abortReasonStr(AbortReason reason)
{
  switch (reason) {
    case AbortReason::Other:
      return "other";
    case AbortReason::ModeFailure:
      return "modeFailure";
    case AbortReason::TrajectoryFailure:
      return "trajectoryFailure";
    case AbortReason::NoValidMission:
      return "noValidMission";
    case AbortReason::ActionDoesNotExist:
      return "actionDoesNotExist";
  }
  return "unknown";
}

void MissionExecutor::checkArmingAndRunConditions(HealthAndArmingCheckReporter & reporter)
{
  // TODO: report readyness
  if (!isReady()) {
  }
}

void MissionExecutor::checkReadynessAndReport()
{
  // Only while not active
  if (_is_active) {
    return;
  }

  // Check if actions are ready (use std::set to remove duplicates)
  std::set<std::string> all_action_errors;
  for (const auto & mission_item : _mission->items()) {
    if (const auto * action_item = std::get_if<ActionItem>(&mission_item)) {
      const auto action_iter = _actions.find(action_item->name);
      if (action_iter != _actions.end()) {
        std::vector<std::string> errors;
        if (!action_iter->second->canRun(action_item->arguments, errors)) {
          if (errors.empty()) {
            errors.push_back("Action '" + action_item->name + "' is not ready yet");
          }
          for (const auto & error : errors) {
            all_action_errors.insert(error);
          }
        }
      }
    }
  }
  _actions_ready = all_action_errors.empty();

  if (_on_readyness_update) {
    std::vector all_errors = _mission_errors;
    for (const auto & action_error : all_action_errors) {
      all_errors.push_back(action_error);
    }
    _reporting->callAsync(
      [this, all_errors]
      {
        _on_readyness_update(all_errors.empty(), all_errors);
      });
  }
}

void MissionExecutor::updateSetpoint()
{
  if (_mission_item_state != MissionItemState::Trajectory) {
    // After triggering a mode switch we might still get an update setpoint request until the mode actually switches
    if (!_trajectory_update_warn.has_value()) {
      _trajectory_update_warn = _node.get_clock()->now();
    }
    if (_node.get_clock()->now() > *_trajectory_update_warn + 1s) {
      // The most likely cause of this is an action that did not trigger any action
      RCLCPP_ERROR_THROTTLE(
        _node.get_logger(),
        *_node.get_clock(), 1000, "Trying to update setpoint, but no trajectory set");
    }
    return;
  }
  _trajectory_update_warn.reset();
  _trajectory_executor->updateSetpoint();
}

void MissionExecutor::onActivate()
{
  _is_active = true;
  _trajectory_options = _mission->defaults().trajectory_options;
  _mission_item_state = MissionItemState::Other;
  _active_mode = _mode->id();

  if (!isReady()) {
    RCLCPP_ERROR(
      _node.get_logger(), "Activated without valid mission, or actions not ready, aborting");
    abort(AbortReason::NoValidMission);
    return;
  }

  if (_on_activated) {
    _reporting->callAsync(_on_activated);
  }

  const bool start_from_beginning = !_state.current_index.has_value();
  if (start_from_beginning) {
    setCurrentMissionIndex(0);
  } else {
    // Set the current index (just for progress reporting)
    setCurrentMissionIndex(*_state.current_index);
  }

  if (_actions.find("onResume") == _actions.end() || start_from_beginning) {
    runStoredActions();
    runCurrentMissionItem(!start_from_beginning);
  } else {
    nlohmann::json arguments = getOnResumeStateAndClear();
    arguments["action"] = "resume";
    runAction(
      "onResume", ActionArguments(arguments), [this, action_handler = _action_handler]()
      {
        if (action_handler->isValid()) {
          runStoredActions();
          runCurrentMissionItem(true);
        }
      });
  }
}

void MissionExecutor::onDeactivate(ModeExecutorBase::DeactivateReason reason)
{
  _is_active = false;
  invalidateActionHandler();

  // If landed and at the last item, or the last item is completed, flag the mission as done
  const bool landed = _land_detected->lastValid(3s) && _land_detected->landed();
  if ((landed && _state.current_index == _mission->items().size() - 1) ||
    _state.current_index >= _mission->items().size())
  {
    clearPersistentState();
    resetMissionState();
  } else {
    runOnResumeStoreState();
    savePersistentState();
  }

  deactivateAllActions();

  if (_on_deactivated) {
    _reporting->callAsync(_on_deactivated);
  }
}

void MissionExecutor::runMode(
  ModeBase::ModeID mode_id,
  const std::function<void()> & on_completed, const std::function<void()> & on_failure)
{
  if (!_is_active) {
    RCLCPP_ERROR(_node.get_logger(), "runMode: not active");
    return;
  }
  RCLCPP_DEBUG(_node.get_logger(), "Running mode %i", mode_id);
  _mission_item_state = MissionItemState::Other;
  _active_mode = mode_id;
  _mode_executor->scheduleMode(
    mode_id, [this, on_completed, on_failure, mode_id](const Result & result)
    {
      if (result == Result::Success) {
        on_completed();
      } else if (result != Result::Deactivated) {
        _active_mode = 0; // Reset active mode as we don't know which one is active now
        if (on_failure) {
          on_failure();
        } else {
          RCLCPP_ERROR(
            _node.get_logger(), "Mode %i failed (%s), aborting", mode_id,
            resultToString(result));
          abort(AbortReason::ModeFailure);
        }
      }
    });
}

void MissionExecutor::runAction(
  const std::string & action_name, const ActionArguments & arguments,
  const std::function<void()> & on_completed)
{
  if (!_is_active) {
    RCLCPP_ERROR(_node.get_logger(), "runAction: not active");
    return;
  }

  _mission_item_state = MissionItemState::Other;
  RCLCPP_DEBUG(_node.get_logger(), "Running action '%s'", action_name.c_str());
  auto action_iter = _actions.find(action_name);
  if (action_iter == _actions.end()) {
    RCLCPP_ERROR(_node.get_logger(), "Trying to run nonexistent action '%s'", action_name.c_str());
    abort(AbortReason::ActionDoesNotExist);
    return;
  }
  action_iter->second->run(_action_handler, arguments, on_completed);
}

void MissionExecutor::runTrajectory(
  const std::shared_ptr<Mission> & trajectory, int start_index, int end_index,
  const std::function<void(int)> & on_index_reached, bool stop_at_last_item)
{
  if (!_is_active) {
    RCLCPP_ERROR(_node.get_logger(), "runTrajectory: not active");
    return;
  }

  RCLCPP_DEBUG(
    _node.get_logger(), "Running trajectory (start: %i, end: %i, stop: %i)", start_index,
    end_index, stop_at_last_item);

  const auto on_failure = [&]
    {
      RCLCPP_ERROR(_node.get_logger(), "Triggering abort from trajectory");
      abort(AbortReason::TrajectoryFailure);
    };

  const TrajectoryExecutorInterface::TrajectoryConfig config{trajectory, _trajectory_options,
    start_index, end_index,
    stop_at_last_item, on_index_reached, on_failure};
  _trajectory_executor->runTrajectory(config);
  // Activate mode if not already active
  if (_active_mode != _mode->id()) {
    // We don't need to handle the completion callback, this is done in the trajectory executor
    runMode(_mode->id(), [] {});
  }
  // Set state after changing the mode
  _mission_item_state = MissionItemState::Trajectory;
}

void MissionExecutor::runNextMissionItem()
{
  int index = 0;
  if (_state.current_index.has_value()) {
    index = *_state.current_index + 1;
  }
  setCurrentMissionIndex(index);
  runCurrentMissionItem(false);
}

void MissionExecutor::runCurrentMissionItem(bool resuming)
{
  if (!_state.current_index.has_value()) {
    RCLCPP_ERROR(_node.get_logger(), "No current mission index, entering Hold");
    runMode(ModeBase::kModeIDLoiter, []() {});
    return;
  }
  if (*_state.current_index >= static_cast<int>(_mission->items().size()) ||
    *_state.current_index < 0)
  {
    // Mission completed. If we were running a trajectory, switch into Hold.
    // Otherwise, we assume the previous mode triggered a landing (e.g. via RTL). This is not necessarily true,
    // but we cannot directly look at the landing state here, as it might not be set yet.
    if (_active_mode != _mode->id()) {
      RCLCPP_DEBUG(_node.get_logger(), "Mission completed");
    } else {
      RCLCPP_DEBUG(_node.get_logger(), "Mission completed, entering Hold");
      runMode(ModeBase::kModeIDLoiter, []() {});
    }
    return;
  }
  const auto & item = _mission->items()[*_state.current_index];
  std::visit(
    util::Overloaded{
      [&](const NavigationItem & navigation_item)
      {
        const auto[end_index, should_stop_at_index] = getNextTrajectorySegment(
          *_state.current_index);
        runTrajectory(
          _mission, *_state.current_index, end_index,
          [end_index, this, action_handler = _action_handler](int index)
          {
            // As the callback might still be called after the mission is deactivated or aborted, we check if the previous
            // handler is still valid
            if (action_handler->isValid()) {
              setCurrentMissionIndex(index + 1);
              if (index == end_index) {
                runCurrentMissionItem(false);
              }
            }
          }, should_stop_at_index);
      },
      [&](const ActionItem & action_item)
      {
        nlohmann::json arguments = action_item.arguments.json();
        if (resuming) {
          arguments["internal:resuming"] = true;
        }
        runAction(
          action_item.name, ActionArguments(arguments), [this, action_handler = _action_handler]()
          {
            if (action_handler->isValid()) {
              runNextMissionItem();
            }
          });
      }
    },
    item);

}

void MissionExecutor::resetMissionState()
{
  _state.current_index.reset();
  _state.continuous_actions.clear();
}

std::pair<int, bool> MissionExecutor::getNextTrajectorySegment(int start_index) const
{
  int end_index = start_index;
  bool should_stop_at_end = false;
  while (end_index < static_cast<int>(_mission->items().size()) - 1) {
    // If the next item is an action, determine the stop flag and return
    if (!std::holds_alternative<NavigationItem>(_mission->items()[end_index + 1])) {
      // Check all following action items, and check if any of them wants to stop
      for (int index = end_index + 1; index < static_cast<int>(_mission->items().size()); ++index) {
        if (const auto * action_item = std::get_if<ActionItem>(&_mission->items()[index])) {
          const auto action_iter = _actions.find(action_item->name);
          if (action_iter != _actions.end()) {
            if (action_iter->second->shouldStopAtWaypoint(action_item->arguments)) {
              should_stop_at_end = true;
            }
          }
        } else {
          break;
        }
      }
      break;
    }
    ++end_index;
  }

  // Always stop at end of the mission if it is not an action
  if (end_index == static_cast<int>(_mission->items().size()) - 1) {
    should_stop_at_end = true;
  }

  return {end_index, should_stop_at_end};
}

bool MissionExecutor::currentActionSupportsResumeFromLanded() const
{
  if (!_state.current_index) {
    return false;
  }
  const auto & item = _mission->items()[*_state.current_index];
  if (const auto * action_item = std::get_if<ActionItem>(&item)) {
    const auto action_iter = _actions.find(action_item->name);
    if (action_iter != _actions.end()) {
      return action_iter->second->supportsResumeFromLanded();
    }
  }
  return false;
}

void MissionExecutor::setTrajectoryOptions(const TrajectoryOptions & options)
{
  _trajectory_options.combineWith(options);
}

void MissionExecutor::clearTrajectoryOptions()
{
  _trajectory_options = _mission->defaults().trajectory_options;
}

void MissionExecutor::setCurrentMissionIndex(int index)
{
  _state.current_index = index;
  RCLCPP_DEBUG(_node.get_logger(), "Setting current mission item to %i", *_state.current_index);

  if (index >= static_cast<int>(_mission->items().size())) {
    if (_on_completed) {
      _reporting->callAsync(_on_completed);
    }
  } else {
    if (_on_progress_update) {
      _reporting->callAsync([this, index] {_on_progress_update(index);});
    }
  }
}

void MissionExecutor::abort(AbortReason reason)
{
  invalidateActionHandler();
  const std::string reason_str = abortReasonStr(reason);
  RCLCPP_DEBUG(
    _node.get_logger(), "Aborting mission (reason: %s, recursion level: %i)",
    reason_str.c_str(), _abort_recursion_level);
  if (_abort_recursion_level > 10) {
    // The onFailure action might trigger actions that fail again, leading to recursive calls.
    // So we need to ensure there's a bound on the recursion level.
    // Reaching the limit should generally be fixed in the onFailure action, rather than trying to do anything else here
    RCLCPP_ERROR(_node.get_logger(), "Maximum abort recursion level reached");
    return;
  }

  deactivateAllActions();

  ++_abort_recursion_level;
  if (_actions.find("onFailure") == _actions.end()) {
    RCLCPP_WARN(
      _node.get_logger(),
      "Trying to abort mission, but no onFailure action available. Not doing anything");
  } else {
    nlohmann::json arguments;
    arguments["recursionLevel"] = _abort_recursion_level;
    arguments["reason"] = reason_str;
    if (_state.current_index) {
      arguments["currentIndex"] = _state.current_index.value();
      resetMissionState();
    }
    runAction(
      "onFailure", ActionArguments(arguments), [this]
      {
        RCLCPP_DEBUG(_node.get_logger(), "onFailure action completed");
        // We expect the failure action to handle everything, so do not try to do anything in addition here.
      });
  }
  --_abort_recursion_level;
}

void MissionExecutor::invalidateActionHandler()
{
  _action_handler->setInvalid();
  _action_handler = std::make_shared<ActionHandler>(*this);
}

void ActionHandler::runTrajectory(
  const std::shared_ptr<Mission> & trajectory, const std::function<void()> & on_completed,
  bool stop_at_last_item)
{
  if (!_valid) {
    RCLCPP_WARN(_mission_executor._node.get_logger(), "ActionHandler is not valid anymore");
    return;
  }
  RCLCPP_DEBUG(
    _mission_executor._node.get_logger(), "Running custom trajectory (%lu items)",
    trajectory->items().size());

  if (trajectory->items().empty()) {
    on_completed();
    return;
  }

  // Check if trajectory is valid:
  // - contains no actions
  // - trajectory executor supports navigation types and frame types
  bool failure = false;
  for (const auto & item : trajectory->items()) {
    if (const auto * navigation_item = std::get_if<NavigationItem>(&item)) {
      std::visit(
        util::Overloaded{
          [&](const Waypoint & waypoint)
          {
            if (!_mission_executor._trajectory_executor->navigationItemTypeSupported(
              NavigationItemType::Waypoint))
            {
              RCLCPP_ERROR(
                _mission_executor._node.get_logger(),
                "Trajectory contains navigation type 'waypoint', which is not supported");
              failure = true;
            }
            if (!_mission_executor._trajectory_executor->frameSupported(waypoint.frame)) {
              RCLCPP_ERROR(
                _mission_executor._node.get_logger(),
                "Trajectory contains frame type '%i', which is not supported",
                static_cast<int>(waypoint.frame));
              failure = true;
            }
          }
        }, navigation_item->data);

    } else {
      RCLCPP_ERROR(
        _mission_executor._node.get_logger(),
        "Custom trajectory contains an action item, which is not supported");
      failure = true;
    }
    if (failure) {
      _mission_executor.abort(MissionExecutor::AbortReason::TrajectoryFailure);
      return;
    }
  }

  const int end_index = trajectory->items().size() - 1;
  _mission_executor.runTrajectory(
    trajectory, 0, end_index, [on_completed, end_index](int index)
    {
      if (index == end_index) {
        on_completed();
      }
    }, stop_at_last_item);

}

std::optional<int> ActionHandler::getCurrentMissionIndex() const
{
  return _mission_executor._state.current_index;
}

bool ActionHandler::currentActionSupportsResumeFromLanded() const
{
  return _mission_executor.currentActionSupportsResumeFromLanded();
}

} // namespace px4_ros2
