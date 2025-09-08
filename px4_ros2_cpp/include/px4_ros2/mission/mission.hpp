/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <optional>
#include <thread>
#include <filesystem>
#include <utility>
#include <variant>
#include <Eigen/Eigen>

#include <px4_ros2/third_party/nlohmann/json_fwd.hpp>

#include <rclcpp/rclcpp.hpp>

namespace px4_ros2
{
/** \ingroup mission
 *  @{
 */

/**
 * @brief Arguments passed to an action from the mission JSON definition
 */
class ActionArguments
{
public:
  ActionArguments();

  explicit ActionArguments(const nlohmann::json & json);

  const nlohmann::json & json() const
  {
    return *_data;
  }

  bool contains(const std::string & key) const;

  template<typename T>
  T at(const std::string & key) const;

  /**
   * @brief Check if the action is being resumed
   *
   * This is set when the mission was interrupted and resumed while executing this specific action.
   */
  bool resuming() const;

private:
  std::shared_ptr<nlohmann::json> _data;
};

enum class MissionFrame
{
  Global,   ///< WGS84 latitude/longitude and altitude is mean sea level (MSL), corresponds to MAV_FRAME_GLOBAL
};

static inline std::string missionFrameStr(MissionFrame frame)
{
  switch (frame) {
    case MissionFrame::Global:
      return "global";
  }
  return "unknown";
}

struct Waypoint
{
  Waypoint() = default;
  explicit Waypoint(Eigen::Vector3d a_coordinate, MissionFrame a_frame = MissionFrame::Global)
  : coordinate(std::move(a_coordinate)), frame(a_frame) {}

  Eigen::Vector3d coordinate;

  MissionFrame frame{MissionFrame::Global};

  friend void from_json(const nlohmann::json & j, Waypoint & o); // NOLINT
  friend void to_json(nlohmann::json & j, const Waypoint & o); // NOLINT
};

enum class NavigationItemType
{
  Waypoint,
};

struct NavigationItem
{
  NavigationItem() = default;
  NavigationItem(const Waypoint & waypoint) // NOLINT(google-explicit-constructor)
  : data(waypoint) {}

  std::string id;
  std::variant<Waypoint> data;

  friend void from_json(const nlohmann::json & j, NavigationItem & o); // NOLINT
  friend void to_json(nlohmann::json & j, const NavigationItem & o); // NOLINT
};


struct ActionItem
{
  ActionItem() = default;
  explicit ActionItem(std::string a_name, ActionArguments a_arguments = {})
  : name(std::move(a_name)), arguments(std::move(a_arguments)) {}

  std::string name;
  std::string id;

  ActionArguments arguments;

  friend void from_json(const nlohmann::json & j, ActionItem & o); // NOLINT
  friend void to_json(nlohmann::json & j, const ActionItem & o); // NOLINT
};

using MissionItem = std::variant<NavigationItem, ActionItem>;

struct TrajectoryOptions
{
  std::optional<float> horizontal_velocity; ///< [m/s]
  std::optional<float> vertical_velocity; ///< [m/s]
  std::optional<float> max_heading_rate; ///< [rad/s]

  void combineWith(const TrajectoryOptions & options)
  {
    if (options.horizontal_velocity) {
      horizontal_velocity = options.horizontal_velocity;
    }
    if (options.vertical_velocity) {
      vertical_velocity = options.vertical_velocity;
    }
    if (options.max_heading_rate) {
      max_heading_rate = options.max_heading_rate;
    }
  }

  friend void from_json(const nlohmann::json & j, TrajectoryOptions & o); // NOLINT
  friend void to_json(nlohmann::json & j, const TrajectoryOptions & o); // NOLINT
};

struct MissionDefaults
{
  TrajectoryOptions trajectory_options;

  friend void from_json(const nlohmann::json & j, MissionDefaults & o); // NOLINT
  friend void to_json(nlohmann::json & j, const MissionDefaults & o); // NOLINT
};

/**
 * @brief Mission definition
 *
 * Typically loaded from a JSON file
 * @ingroup mission
 */
class Mission
{
public:
  Mission() = default;

  explicit Mission(
    std::vector<MissionItem> mission_items,
    MissionDefaults mission_defaults = {})
  : _mission_defaults(mission_defaults), _mission_items(std::move(mission_items)) {}

  const MissionDefaults & defaults() const {return _mission_defaults;}

  const std::vector<MissionItem> & items() const {return _mission_items;}

  bool indexValid(int index) const
  {
    return index >= 0 && index < static_cast<int>(_mission_items.size());
  }

  friend void from_json(const nlohmann::json & j, Mission & o); // NOLINT
  friend void to_json(nlohmann::json & j, const Mission & o); // NOLINT

  std::string checksum() const;

private:
  MissionDefaults _mission_defaults;
  std::vector<MissionItem> _mission_items;
};

/**
 * @brief File monitor to load a mission from a JSON file
 * @ingroup mission
 */
class MissionFileMonitor
{
public:
  MissionFileMonitor(
    std::shared_ptr<rclcpp::Node> node, std::filesystem::path filename,
    std::function<void(std::shared_ptr<Mission>)> on_mission_update);
  ~MissionFileMonitor();

private:
  void run();
  void fileUpdated();

  std::shared_ptr<rclcpp::Node> _node;
  const std::filesystem::path _filename;
  const std::function<void(std::shared_ptr<Mission>)> _on_mission_update;
  std::thread _thread;
  int _event_fd{-1};
  rclcpp::TimerBase::SharedPtr _update_timer;
};


/** @}*/
} /* namespace px4_ros2 */
