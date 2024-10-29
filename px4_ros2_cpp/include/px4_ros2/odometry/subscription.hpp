/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_ros2/common/context.hpp>

using namespace std::chrono_literals; // NOLINT

namespace px4_ros2
{
/** \ingroup odometry
 *  @{
 */

/**
 * @brief Provides a subscription to arbitrary ROS topics.
 */
template<typename RosMessageType>
class Subscription
{
  using UpdateCallback = std::function<void (const RosMessageType &)>;

public:
  Subscription(Context & context, const std::string & topic)
  : _node(context.node())
  {
    const std::string namespaced_topic = context.topicNamespacePrefix() + topic;
    _subscription = _node.create_subscription<RosMessageType>(
      namespaced_topic, rclcpp::QoS(1).best_effort(),
      [this](const typename RosMessageType::UniquePtr msg) {
        _last = *msg;
        _last_message_time = _node.get_clock()->now();
        for (const auto & callback : _callbacks) {
          callback(_last);
        }
      });
  }

  /**
   * @brief Add a callback to execute when receiving a new message.
   *
   * @param callback the callback function
   */
  void onUpdate(const UpdateCallback & callback)
  {
    _callbacks.push_back(callback);
  }

  /**
   * @brief Get the last-received message.
   *
   * @returns the last-received ROS message
   * @throws std::runtime_error when no messages have been received
   */
  const RosMessageType & last() const
  {
    if (!hasReceivedMessages()) {
      throw std::runtime_error("No messages received.");
    }
    return _last;
  }

  /**
   * @brief Get the receive-time of the last message.
   *
   * @returns the time at which the last ROS message was received
   */
  const rclcpp::Time & lastTime() const
  {
    return _last_message_time;
  }

  /**
   * @brief Check whether the last message is still valid.
   * To be valid, the message must have been received within a given time of the current time.
   *
   * @param max_delay the maximum delay between the current time and when the message was received
   * @return true if the last message was received within the maximum delay
   */
  template<typename DurationT = std::milli>
  bool lastValid(const std::chrono::duration<int64_t, DurationT> max_delay = 500ms) const
  {
    return hasReceivedMessages() && _node.get_clock()->now() - _last_message_time < max_delay;
  }

protected:
  rclcpp::Node & _node;

private:
  typename rclcpp::Subscription<RosMessageType>::SharedPtr _subscription{nullptr};

  RosMessageType _last;
  rclcpp::Time _last_message_time;

  std::vector<std::function<void(const RosMessageType &)>> _callbacks{};

  bool hasReceivedMessages() const
  {
    return _last_message_time.seconds() != 0;
  }
};

/** @}*/
} // namespace px4_ros2
