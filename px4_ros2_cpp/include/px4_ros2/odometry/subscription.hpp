/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_ros2/common/context.hpp>

namespace px4_ros2
{
/** \ingroup odometry
 *  @{
 */

/**
 * Provides a subscription to arbitrary ROS topics.
 */
template<typename RosMessageType>
class Subscription
{
public:
  Subscription(Context & context, const std::string & topic)
  : _node(context.node()), _topic(context.topicNamespacePrefix() + topic)
  {
    _subscription = _node.create_subscription<RosMessageType>(
      _topic, rclcpp::QoS(1).best_effort(),
      [this](const typename RosMessageType::UniquePtr msg) {
        _last = *msg;
        _last_message_time = _node.get_clock()->now();
        for (const auto & callback : _callbacks) {
          callback(_last);
        }
      });
  }

  /**
   * @brief Register a callback to be executed for each message.
   *
   * @param callback the callback function
   */
  void subscribe(std::function<void(RosMessageType)> callback)
  {
    _callbacks.push_back(callback);
  }

  /**
   * @brief Get the last-received message.
   *
   * @returns the last-received ROS message
   * @throws std::runtime_error when no messages have been received
   */
  RosMessageType last() const
  {
    if (_last_message_time.seconds() == 0) {
      throw std::runtime_error("No messages received.");
    }
    return _last;
  }

private:
  rclcpp::Node & _node;
  std::string _topic;

  typename rclcpp::Subscription<RosMessageType>::SharedPtr _subscription{nullptr};

  RosMessageType _last;
  rclcpp::Time _last_message_time;

  std::vector<std::function<void(RosMessageType)>> _callbacks{};
};

/** @}*/
} // namespace px4_ros2
