/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "px4_sdk/components/manual_control_input.hpp"
#include "px4_sdk/components/mode.hpp"

namespace px4_sdk
{

ManualControlInput::ManualControlInput(ModeBase & mode_base)
: _node(mode_base.node())
{
  _manual_control_setpoint.set__valid(false);

  _manual_control_setpoint_sub =
    mode_base.node().create_subscription<px4_msgs::msg::ManualControlSetpoint>(
    mode_base.topicNamespacePrefix() + "/fmu/out/manual_control_setpoint", rclcpp::QoS(
      1).best_effort(),
    [this](px4_msgs::msg::ManualControlSetpoint::UniquePtr msg) {
      _manual_control_setpoint = *msg;
      _last_manual_control_setpoint = _node.get_clock()->now();
    });
}
} // namespace px4_sdk
