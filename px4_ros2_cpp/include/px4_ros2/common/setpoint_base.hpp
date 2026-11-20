/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <px4_msgs/msg/setpoint_config.hpp>
#include <px4_msgs/msg/setpoint_config_reply.hpp>
#include <rclcpp/rclcpp.hpp>

#include "context.hpp"
#include "exception.hpp"

namespace px4_ros2 {

class SetpointBase : public std::enable_shared_from_this<SetpointBase> {
 public:
  using ShouldActivateCB = std::function<void()>;
  using SetpointType = decltype(px4_msgs::msg::SetpointConfig::type);

  explicit SetpointBase(Context& context) { context.addSetpointType(this); }

  virtual ~SetpointBase() = default;

  std::shared_ptr<SetpointBase> getSharedPtr()
  {
    try {
      return shared_from_this();
    } catch (const std::bad_weak_ptr& exception) {
      throw Exception("Setpoint must be instantiated with std::make_shared<>");
    }
    return {};
  }

  /**
   * Returns one of px4_msgs::msg::SetpointType::TYPE_*
   */
  virtual SetpointType getSetpointType() = 0;

  /**
   * Allows a setpoint class to clear an optional requirement. This is for setpoint types that
   * support multiple variations, for example some that require local position and others that do
   * not.
   *
   * @param setpoint_config_reply input and output config
   */
  virtual void clearOptionalRequirements(px4_msgs::msg::SetpointConfigReply& setpoint_config_reply)
  {
  }

  virtual float desiredUpdateRateHz() { return 50.f; }

  /**
   * Setpoint timeout. If the mode is active with this setpoint type, the vehicle is armed and not
   * landed, and the FMU receives no setpoint for longer than this, the FMU handles the mode like
   * an unresponsive one (a failsafe, or the internal mode for a replacement). Use several update
   * periods of the setpoint (a timeout shorter than the update interval of the mode logs a
   * warning). 0 or a negative value disables it (default), values above 65535 ms are limited to
   * that. FMU versions without support ignore it.
   *
   * A change while the setpoint type is in use is sent with the next setpoint update.
   * ModeBase::completed() sets the timeout back to 0 until the mode gets activated again, so the
   * mode can stop sending setpoints once it completed.
   */
  void setTimeout(std::chrono::milliseconds timeout)
  {
    if (timeout != _timeout) {
      _timeout = timeout;
      _active = false;  // The next update sends the configuration again
    }
  }
  std::chrono::milliseconds timeout() const { return _timeout; }

  void setShouldActivateCallback(const ShouldActivateCB& should_activate_cb)
  {
    _should_activate_cb = should_activate_cb;
  }
  void setActive(bool active) { _active = active; }

 protected:
  void onUpdate()
  {
    if (!_active && _should_activate_cb) {
      _should_activate_cb();
    }
  }

 private:
  ShouldActivateCB _should_activate_cb;
  bool _active{false};
  std::chrono::milliseconds _timeout{0};
};

} /* namespace px4_ros2 */
