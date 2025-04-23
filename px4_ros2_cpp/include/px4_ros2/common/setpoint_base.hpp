/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include "context.hpp"

namespace px4_ros2
{

class SetpointBase : public std::enable_shared_from_this<SetpointBase>
{
public:
  using ShouldActivateCB = std::function<void ()>;

  struct Configuration
  {
    void fillControlMode(px4_msgs::msg::VehicleControlMode & control_mode)
    {
      control_mode.flag_control_rates_enabled = rates_enabled;
      control_mode.flag_control_attitude_enabled = attitude_enabled;
      control_mode.flag_control_acceleration_enabled = acceleration_enabled;
      control_mode.flag_control_velocity_enabled = velocity_enabled;
      control_mode.flag_control_position_enabled = position_enabled;
      control_mode.flag_control_altitude_enabled = altitude_enabled;
      control_mode.flag_control_allocation_enabled = control_allocation_enabled;
      control_mode.flag_control_climb_rate_enabled = climb_rate_enabled;
    }

    bool control_allocation_enabled{true};
    bool rates_enabled{true};
    bool attitude_enabled{true};
    bool altitude_enabled{true};
    bool acceleration_enabled{true};
    bool velocity_enabled{true};
    bool position_enabled{true};
    bool local_position_is_optional{false};
    bool climb_rate_enabled{false};
  };

  explicit SetpointBase(Context & context)
  {
    context.addSetpointType(this);
  }

  virtual ~SetpointBase() = default;

  std::shared_ptr<SetpointBase> getSharedPtr()
  {
    try {
      return shared_from_this();
    } catch (const std::bad_weak_ptr & exception) {
      throw std::runtime_error("Setpoint must be instantiated with std::make_shared<>");
    }
    return {};
  }

  virtual Configuration getConfiguration() = 0;

  virtual float desiredUpdateRateHz() {return 50.f;}


  void setShouldActivateCallback(const ShouldActivateCB & should_activate_cb)
  {
    _should_activate_cb = should_activate_cb;
  }
  void setActive(bool active) {_active = active;}

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
};

} /* namespace px4_ros2 */
