/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <fstream>
#include <unistd.h>
#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/visit.hpp>

#include <px4_ros2/third_party/nlohmann/json.hpp>

#include <poll.h>
#include <sys/types.h>
#include <sys/inotify.h>
#include <sys/eventfd.h>

using namespace std::chrono_literals; // NOLINT

namespace px4_ros2
{
ActionArguments::ActionArguments()
: _data(std::make_shared<nlohmann::json>())
{
}
ActionArguments::ActionArguments(const nlohmann::json & json)
: _data(std::make_shared<nlohmann::json>(json))
{
}
template<typename T>
T ActionArguments::at(const std::string & key) const
{
  if (!contains(key)) {
    throw std::out_of_range("Key not found: " + key);
  }
  return _data->at(key).get<T>();
}

template char ActionArguments::at(const std::string & key) const;
template int ActionArguments::at<int>(const std::string & key) const;
template unsigned ActionArguments::at<unsigned>(const std::string & key) const;
template long ActionArguments::at<long>(const std::string & key) const;
template unsigned long ActionArguments::at<unsigned long>(const std::string & key) const;
template bool ActionArguments::at<bool>(const std::string & key) const;
template double ActionArguments::at<double>(const std::string & key) const;
template float ActionArguments::at<float>(const std::string & key) const;
template std::string ActionArguments::at<std::string>(const std::string & key) const;
template std::vector<int> ActionArguments::at<std::vector<int>>(const std::string & key) const;
template std::vector<bool> ActionArguments::at<std::vector<bool>>(const std::string & key) const;
template std::vector<std::string> ActionArguments::at<std::vector<std::string>>(
  const std::string & key) const;

bool ActionArguments::contains(const std::string & key) const
{
  return _data->contains(key);
}

bool ActionArguments::resuming() const
{
  return contains("internal:resuming") && at<bool>("internal:resuming");
}

std::string Mission::checksum() const
{
  try {
    const nlohmann::json json = *this;
    // Note: the hash is platform-specific
    return std::to_string(std::hash<nlohmann::json>{}(json));
  } catch (const std::exception & e) {
    printf("Failed trying to parse mission: %s\n", e.what());
  }
  return "";
}

MissionFileMonitor::MissionFileMonitor(
  std::shared_ptr<rclcpp::Node> node, std::filesystem::path filename,
  std::function<void(std::shared_ptr<Mission>)> on_mission_update)
: _node(std::move(node)), _filename(std::move(filename)),
  _on_mission_update(std::move(on_mission_update))
{
  _event_fd = eventfd(0, 0);
  if (_event_fd < 0) {
    RCLCPP_ERROR(_node->get_logger(), "eventfd() failed (%s)", strerror(errno));
  }
  _update_timer = _node->create_wall_timer(
    1ms, [this]
    {
      _update_timer->cancel();
      fileUpdated();
    });
  _update_timer->cancel();

  _thread = std::thread(&MissionFileMonitor::run, this);
}

MissionFileMonitor::~MissionFileMonitor()
{
  // Wake up thread
  uint64_t value = 1;
  const int ret = write(_event_fd, &value, sizeof(value));
  if (ret > 0 && _thread.joinable()) {
    _thread.join();
  }
  close(_event_fd);
}

void MissionFileMonitor::run()
{
  constexpr int kBufLen = 16 * (sizeof(inotify_event) + 16);
  const int fd = inotify_init();

  if (fd < 0) {
    RCLCPP_ERROR(_node->get_logger(), "inotify_init() failed (%s)", strerror(errno));
    return;
  }

  char buffer[kBufLen];
  const int wd = inotify_add_watch(
    fd, _filename.parent_path().c_str(),
    IN_MOVED_TO | IN_CLOSE_WRITE);

  // Immediately notify if the file already exists
  std::error_code ec;
  if (std::filesystem::exists(_filename, ec)) {
    // Async update notification
    _update_timer->reset();
  }

  pollfd fds[2];
  fds[0].fd = _event_fd;
  fds[0].events = POLLIN;
  fds[1].fd = fd;
  fds[1].events = POLLIN;

  while (true) {
    const int ret = poll(fds, 2, -1);
    if (ret == 0) {
      // Timeout, not expected
      continue;
    }
    if (ret < 0) {
      RCLCPP_ERROR_ONCE(_node->get_logger(), "poll() failed (%s)", strerror(errno));
      break;
    }

    if (fds[0].revents & POLLIN) {
      // Event fd -> exit
      break;
    }

    if (fds[1].revents & POLLIN) {
      // Notification fd
      const int length = read(fd, buffer, kBufLen);

      if (length < 0) {
        RCLCPP_ERROR(_node->get_logger(), "read() failed (%s)", strerror(errno));
        break;
      }

      int i = 0;
      while (i < length) {
        const inotify_event * event = reinterpret_cast<struct inotify_event *>(&buffer[i]);
        if (event->len && wd == event->wd && _filename.filename() == event->name) {
          // Async update notification
          _update_timer->reset();
        }
        i += sizeof(inotify_event) + event->len;
      }
    }
  }

  inotify_rm_watch(fd, wd);
  close(fd);
}

void MissionFileMonitor::fileUpdated()
{
  try {
    std::ifstream f(_filename);
    const nlohmann::json data = nlohmann::json::parse(f);
    _on_mission_update(std::make_shared<Mission>(data));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(_node->get_logger(), "Failed trying to parse mission file: %s", e.what());
  }
}

void to_json(nlohmann::json & j, const Mission & o)
{
  nlohmann::json::array_t items;
  for (const auto & item : o.items()) {
    std::visit(
      util::Overloaded{
        [&](const auto & item_t)
        {
          items.push_back(item_t);
        }
      }, item);
  }
  j["version"] = 1;
  j["mission"] = {
    {"defaults", o.defaults()},
    {"items", items},
  };
}

void from_json(const nlohmann::json & j, TrajectoryOptions & o)
{
  if (j.contains("horizontalVelocity")) {
    o.horizontal_velocity = j.at("horizontalVelocity").get<float>();
  }
  if (j.contains("verticalVelocity")) {
    o.vertical_velocity = j.at("verticalVelocity").get<float>();
  }
  if (j.contains("maxHeadingRate")) {
    o.max_heading_rate = degToRad(j.at("maxHeadingRate").get<float>());
  }
}
void to_json(nlohmann::json & j, const TrajectoryOptions & o)
{
  if (o.horizontal_velocity) {
    j["horizontalVelocity"] = *o.horizontal_velocity;
  }
  if (o.vertical_velocity) {
    j["verticalVelocity"] = *o.vertical_velocity;
  }
  if (o.max_heading_rate) {
    j["maxHeadingRate"] = radToDeg(*o.max_heading_rate);
  }
}

void from_json(const nlohmann::json & j, MissionDefaults & o)
{
  o.trajectory_options = j.get<TrajectoryOptions>();
}
void to_json(nlohmann::json & j, const MissionDefaults & o)
{
  const nlohmann::json trajectory_json = o.trajectory_options;
  if (!trajectory_json.is_null()) {
    j.update(trajectory_json);
  }
}

void from_json(const nlohmann::json & j, ActionItem & o)
{
  o.name = j.at("type").get<std::string>();
  if (j.contains("id")) {
    o.id = j.at("id").get<std::string>();
  }
  o.arguments = ActionArguments(j);
}
void to_json(nlohmann::json & j, const ActionItem & o)
{
  j["type"] = o.name;
  if (!o.id.empty()) {
    j["id"] = o.id;
  }
  const auto arguments = o.arguments.json();
  if (!arguments.is_null()) {
    j.update(o.arguments.json());
  }
}
void from_json(const nlohmann::json & j, NavigationItem & o)
{
  if (j.contains("id")) {
    o.id = j.at("id").get<std::string>();
  }
  const auto navigation_type = j.at("navigationType").get<std::string>();
  if (navigation_type == "waypoint") {
    o.data = j.get<Waypoint>();
  } else {
    throw nlohmann::json::other_error::create(0, "Unknown navigation type: " + navigation_type, &j);
  }
}
void to_json(nlohmann::json & j, const NavigationItem & o)
{
  if (!o.id.empty()) {
    j["id"] = o.id;
  }
  std::visit(
    util::Overloaded{
      [&](const Waypoint & waypoint)
      {
        j["type"] = "navigation";
        j["navigationType"] = "waypoint";
        to_json(j, waypoint);
      }
    },
    o.data);
}
void from_json(const nlohmann::json & j, Waypoint & o)
{
  const auto frame = j.at("frame").get<std::string>();
  if (frame == "global") {
    o.frame = MissionFrame::Global;
  } else {
    throw nlohmann::json::other_error::create(0, "Unknown frame: " + frame, &j);
  }
  o.coordinate(0) = j.at("x").get<double>();
  o.coordinate(1) = j.at("y").get<double>();
  o.coordinate(2) = j.at("z").get<double>();
}
void to_json(nlohmann::json & j, const Waypoint & o)
{
  switch (o.frame) {
    case MissionFrame::Global:
      j["frame"] = "global";
      break;
  }
  j["x"] = o.coordinate(0);
  j["y"] = o.coordinate(1);
  j["z"] = o.coordinate(2);

}

void from_json(const nlohmann::json & j, Mission & o)
{
  const int version = j.at("version").get<int>();
  if (version != 1) {
    throw nlohmann::json::other_error::create(
            0,
            "Unsupported mission version: " + std::to_string(version), &j);
  }

  const auto & mission = j.at("mission");
  if (mission.contains("defaults")) {
    o._mission_defaults = mission.at("defaults").get<MissionDefaults>();
  }
  for (const auto & item : mission.at("items")) {
    const std::string type = item.at("type").get<std::string>();
    if (type == "navigation") {
      o._mission_items.emplace_back(item.get<NavigationItem>());
    } else {
      o._mission_items.emplace_back(item.get<ActionItem>());
    }
  }
}

} // namespace px4_ros2
