/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <mutex>
#include <memory>


/**
 * @brief Handle for a registered subscription callback instance.
 *
 * Make sure to keep this stored for as long as the callback should be triggered.
 *
 * When a callback is registered with a `SharedSubscription`, a unique_ptr of this type
 * is returned. The pointer holds the callback token internally, and its custom
 * deleter automatically unregisters the callback when the pointer is destroyed.
 *
 * This ensures that callbacks are safely removed without requiring manual cleanup.
 */
using SharedSubscriptionCallbackInstance = std::unique_ptr<const unsigned,
    std::function<void (const unsigned *)>>;

/**
 * @brief Shared class for a ROS subscription
 *
 * Can be used for consistent callback ordering, by sharing a subscription instance, and
 * to reduce resource (CPU load) overhead from creating multiple subscriptions in different components.
 *
 * When using multiple nodes, a subscription per node is created.
 *
 * Assumes the use of a single-threaded executor.
 */
template<typename RosMessageType>
class SharedSubscription : public std::enable_shared_from_this<SharedSubscription<RosMessageType>>
{
public:
  using OnUpdateCallback =
    std::function<void (const typename RosMessageType::UniquePtr &)>;

  static SharedSubscription & instance(
    rclcpp::Node & node,
    const std::string & topic_name);

  [[nodiscard]] static SharedSubscriptionCallbackInstance create(
    rclcpp::Node & node, const std::string & topic_name, OnUpdateCallback callback)
  {
    return instance(node, topic_name).registerOnUpdateCallback(callback);
  }

  [[nodiscard]] SharedSubscriptionCallbackInstance registerOnUpdateCallback(
    OnUpdateCallback callback);

  const typename rclcpp::Subscription<RosMessageType>::SharedPtr & getSubscription() const
  {
    return _subscription;
  }

private:
  explicit SharedSubscription(
    rclcpp::Node & node,
    const std::string & topic_name);
  void unregisterOnUpdateCallback(unsigned token);
  static void cleanupInstanceIfUnused(const std::shared_ptr<SharedSubscription> & instance);

  void onUpdate(const typename RosMessageType::UniquePtr & msg) const;

  typename rclcpp::Subscription<RosMessageType>::SharedPtr _subscription;
  rclcpp::Node & _node;
  unsigned _next_token{1};
  std::map<unsigned, OnUpdateCallback> _callbacks; // Needs guaranteed ordering, for consistent callback ordering

  struct Key
  {
    rclcpp::Node * node;
    std::string topic_name;
    bool operator<(const Key & other) const
    {
      return node < other.node || topic_name < other.topic_name;
    }
  };

  static std::map<Key, std::shared_ptr<SharedSubscription>> instances;
  static std::mutex mutex;
};

template<typename RosMessageType>
std::map<typename SharedSubscription<RosMessageType>::Key,
  std::shared_ptr<SharedSubscription<RosMessageType>>> SharedSubscription<RosMessageType>::instances;
template<typename RosMessageType>
std::mutex SharedSubscription<RosMessageType>::mutex;

template<typename RosMessageType>
SharedSubscription<RosMessageType> & ::SharedSubscription<RosMessageType>::instance(
  rclcpp::Node & node,
  const std::string & topic_name)
{
  // Maintain a single instance per node and topic name.
  // Even though the ROS MultiThreadedExecutor is not supported by the library, a user might still use multiple Nodes
  // running in different threads. Therefore, we use a mutex here.
  const std::lock_guard lg{mutex};
  const Key key{&node, topic_name};
  auto it = instances.find(key);
  if (it != instances.end()) {
    return *it->second;
  }
  const auto instance =
    std::shared_ptr<SharedSubscription>(new SharedSubscription<RosMessageType>(node, topic_name));
  instances.emplace(key, instance);
  return *instance;
}

template<typename RosMessageType>
SharedSubscriptionCallbackInstance SharedSubscription<RosMessageType>::registerOnUpdateCallback(
  OnUpdateCallback callback)
{
  const unsigned token = _next_token++;
  const auto insert_ret = _callbacks.emplace(token, std::move(callback));
  assert(insert_ret.second);
  (void)insert_ret;
  std::weak_ptr instance = this->shared_from_this();
  return SharedSubscriptionCallbackInstance(
    new unsigned(token), [instance](const unsigned * token)
    {
      const auto instance_shared = instance.lock();
      if (token && instance_shared) {
        instance_shared->unregisterOnUpdateCallback(*token);
        SharedSubscription::cleanupInstanceIfUnused(instance_shared);
      }
    });
}

template<typename RosMessageType>
SharedSubscription<RosMessageType>::SharedSubscription(
  rclcpp::Node & node,
  const std::string & topic_name)
: _node(node)
{
  _subscription = _node.create_subscription<RosMessageType>(
    topic_name, rclcpp::QoS(
      1).best_effort(),
    [this](typename RosMessageType::UniquePtr msg) {
      onUpdate(msg);
    });
}

template<typename RosMessageType>
void SharedSubscription<RosMessageType>::unregisterOnUpdateCallback(unsigned token)
{
  const auto num_erased = _callbacks.erase(token);
  assert(num_erased == 1);
  (void)num_erased;
}

template<typename RosMessageType>
void SharedSubscription<RosMessageType>::cleanupInstanceIfUnused(
  const std::shared_ptr<SharedSubscription> & instance)
{
  // Remove the instance from the map if it is not used anymore.
  // This would not strictly be required, as the static object gets destroyed at program exit. However, this causes
  // segfaults, see https://github.com/ros2/rmw_fastrtps/pull/770 (fixed in ROS Jazzy)
  const std::lock_guard lg{mutex};
  if (instance->_callbacks.empty()) {
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

template<typename RosMessageType>
void SharedSubscription<RosMessageType>::onUpdate(const typename RosMessageType::UniquePtr & msg)
const
{
  for (const auto & cb : _callbacks) {
    cb.second(msg);
  }
}
