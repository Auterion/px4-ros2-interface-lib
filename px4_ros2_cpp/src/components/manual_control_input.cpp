/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "px4_ros2/components/manual_control_input.hpp"
#include "px4_ros2/components/mode.hpp"
#include "px4_ros2/utils/message_version.hpp"

namespace px4_ros2
{

ManualControlInput::ManualControlInput(Context & context, bool is_optional)
: _node(context.node())
{
  _manual_control_setpoint.set__valid(false);

  _manual_control_setpoint_sub =
    context.node().create_subscription<px4_msgs::msg::ManualControlSetpoint>(
    context.topicNamespacePrefix() + "fmu/out/manual_control_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::ManualControlSetpoint>(), rclcpp::QoS(
      1).best_effort(),
    [this](px4_msgs::msg::ManualControlSetpoint::UniquePtr msg) {
      _manual_control_setpoint = *msg;
      _last_manual_control_setpoint = _node.get_clock()->now();
    });

  if (!is_optional) {
    RequirementFlags requirements{};
    requirements.manual_control = true;
    context.setRequirement(requirements);
  }
}

} // namespace px4_ros2
