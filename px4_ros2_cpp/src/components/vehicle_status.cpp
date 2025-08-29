/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "vehicle_status.hpp"

#include <px4_ros2/utils/message_version.hpp>

std::map<SharedVehicleStatus::Key,
  std::shared_ptr<SharedVehicleStatus>> SharedVehicleStatus::instances;
std::mutex SharedVehicleStatus::mutex;

SharedVehicleStatus & SharedVehicleStatus::instance(
  rclcpp::Node & node,
  const std::string & topic_namespace_prefix)
{
  // Maintain a single instance per node and topic namespace prefix.
  // Even though the ROS MultiThreadedExecutor is not supported by the library, a user might still use multiple Nodes
  // running in different threads. Therefore, we use a mutex here.
  const std::lock_guard lg{mutex};
  const Key key{&node, topic_namespace_prefix};
  auto it = instances.find(key);
  if (it != instances.end()) {
    return *it->second;
  }
  const auto status =
    std::shared_ptr<SharedVehicleStatus>(new SharedVehicleStatus(node, topic_namespace_prefix));
  instances.emplace(key, status);
  return *status;
}

SharedVehicleStatusToken SharedVehicleStatus::registerVehicleStatusUpdatedCallback(
  VehicleStatusUpdatedCallback callback)
{
  const unsigned token = _next_token++;
  const auto insert_ret = _vehicle_status_updated_callbacks.emplace(token, std::move(callback));
  assert(insert_ret.second);
  return SharedVehicleStatusToken(shared_from_this(), token);
}

SharedVehicleStatus::SharedVehicleStatus(
  rclcpp::Node & node,
  const std::string & topic_namespace_prefix)
: _node(node)
{
  _vehicle_status_sub = _node.create_subscription<px4_msgs::msg::VehicleStatus>(
    topic_namespace_prefix + "fmu/out/vehicle_status" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(), rclcpp::QoS(
      1).best_effort(),
    [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
      vehicleStatusUpdated(msg);
    });
}

void SharedVehicleStatus::unregisterVehicleStatusUpdatedCallback(unsigned token)
{
  const auto num_erased = _vehicle_status_updated_callbacks.erase(token);
  assert(num_erased == 1);
}

void SharedVehicleStatus::cleanupInstanceIfUnused(
  const std::shared_ptr<SharedVehicleStatus> & instance)
{
  // Remove the instance from the map if it is not used anymore.
  // This would not strictly be required, as the static object gets destroyed at program exit. However, this causes
  // segfaults, see https://github.com/ros2/rmw_fastrtps/pull/770 (fixed in ROS Jazzy)
  const std::lock_guard lg{mutex};
  if (instance->_vehicle_status_updated_callbacks.empty()) {
    auto result = std::find_if(
      instances.begin(), instances.end(),
      [&instance](const auto & obj) {return obj.second == instance;});
    if (result != instances.end()) {
      instances.erase(result);
    } else {
      RCLCPP_FATAL(instance->_node.get_logger(), "Instance not found");
    }
  }
}

void SharedVehicleStatus::vehicleStatusUpdated(const px4_msgs::msg::VehicleStatus::UniquePtr & msg)
const
{
  for (const auto & cb : _vehicle_status_updated_callbacks) {
    cb.second(msg);
  }
}
