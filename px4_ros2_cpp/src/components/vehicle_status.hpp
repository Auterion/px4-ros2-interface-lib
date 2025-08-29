/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <functional>
#include <mutex>
#include <memory>

class SharedVehicleStatusToken;

/**
 * @brief Shared class for vehicle status subscription
 *
 * Can be used for consistent callback ordering, by sharing a subscription instance for multiple modes.
 */
class SharedVehicleStatus : public std::enable_shared_from_this<SharedVehicleStatus>
{
public:
  using VehicleStatusUpdatedCallback =
    std::function<void (const px4_msgs::msg::VehicleStatus::UniquePtr &)>;

  static SharedVehicleStatus & instance(
    rclcpp::Node & node,
    const std::string & topic_namespace_prefix = "");

  [[nodiscard]] SharedVehicleStatusToken registerVehicleStatusUpdatedCallback(
    VehicleStatusUpdatedCallback callback);

  const rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr & getSubscription() const
  {
    return _vehicle_status_sub;
  }

private:
  friend class SharedVehicleStatusToken;

  explicit SharedVehicleStatus(
    rclcpp::Node & node,
    const std::string & topic_namespace_prefix = "");
  void unregisterVehicleStatusUpdatedCallback(unsigned token);
  static void cleanupInstanceIfUnused(const std::shared_ptr<SharedVehicleStatus> & instance);

  void vehicleStatusUpdated(const px4_msgs::msg::VehicleStatus::UniquePtr & msg) const;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
  rclcpp::Node & _node;
  unsigned _next_token{1};
  std::map<unsigned, VehicleStatusUpdatedCallback> _vehicle_status_updated_callbacks; // Needs guaranteed ordering

  struct Key
  {
    rclcpp::Node * node;
    std::string topic_namespace_prefix;
    bool operator<(const Key & other) const
    {
      return node < other.node || topic_namespace_prefix < other.topic_namespace_prefix;
    }
  };

  static std::map<Key, std::shared_ptr<SharedVehicleStatus>> instances;
  static std::mutex mutex;
};

class SharedVehicleStatusToken
{
public:
  SharedVehicleStatusToken(const SharedVehicleStatusToken & other) = delete;
  SharedVehicleStatusToken & operator=(const SharedVehicleStatusToken & other) = delete;

  SharedVehicleStatusToken(SharedVehicleStatusToken && other) noexcept
  : _instance(std::move(other._instance)), _value(other._value)
  {
    other._value = std::nullopt;
    other._instance.reset();
  }
  SharedVehicleStatusToken & operator=(SharedVehicleStatusToken && other) noexcept
  {
    if (this != &other) {
      _value = std::move(other._value);
      other._value = std::nullopt;
      _instance = other._instance;
    }
    return *this;
  }
  ~SharedVehicleStatusToken()
  {
    const auto instance = _instance.lock();
    if (_value.has_value() && instance) {
      instance->unregisterVehicleStatusUpdatedCallback(*_value);
      SharedVehicleStatus::cleanupInstanceIfUnused(instance);
    }
  }

private:
  SharedVehicleStatusToken(const std::shared_ptr<SharedVehicleStatus> & instance, unsigned value)
  : _instance(instance), _value(value) {}
  std::weak_ptr<SharedVehicleStatus> _instance;
  std::optional<unsigned> _value;
  friend class SharedVehicleStatus;
};
