/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/utils/message_version.hpp>


namespace px4_ros2
{

GotoSetpointType::GotoSetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _goto_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::GotoSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/goto_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::GotoSetpoint>(),
    1);
}

void GotoSetpointType::update(
  const Eigen::Vector3f & position,
  const std::optional<float> & heading,
  const std::optional<float> & max_horizontal_speed,
  const std::optional<float> & max_vertical_speed,
  const std::optional<float> & max_heading_rate)
{
  onUpdate();

  px4_msgs::msg::GotoSetpoint sp{};

  // setpoints
  sp.position[0] = position(0);
  sp.position[1] = position(1);
  sp.position[2] = position(2);
  sp.heading = heading.value_or(0.f);

  // setpoint flags
  sp.flag_control_heading = heading.has_value();

  // constraints
  sp.max_horizontal_speed = max_horizontal_speed.value_or(0.f);
  sp.max_vertical_speed = max_vertical_speed.value_or(0.f);
  sp.max_heading_rate = max_heading_rate.value_or(0.f);

  // constraint flags
  sp.flag_set_max_horizontal_speed = max_horizontal_speed.has_value();
  sp.flag_set_max_vertical_speed = max_vertical_speed.has_value();
  sp.flag_set_max_heading_rate = max_heading_rate.has_value();

  sp.timestamp = 0; // Let PX4 set the timestamp
  _goto_setpoint_pub->publish(sp);
}

SetpointBase::Configuration GotoSetpointType::getConfiguration()
{
  Configuration config{};
  config.control_allocation_enabled = true;
  config.rates_enabled = true;
  config.attitude_enabled = true;
  config.altitude_enabled = true;
  config.acceleration_enabled = true;
  config.velocity_enabled = true;
  config.position_enabled = true;
  config.climb_rate_enabled = true;
  return config;
}

GotoGlobalSetpointType::GotoGlobalSetpointType(Context & context)
: _node(context.node()), _map_projection(std::make_unique<MapProjection>(context)),
  _goto_setpoint(std::make_shared<GotoSetpointType>(context))
{
  RequirementFlags requirements{};
  requirements.global_position = true;
  context.setRequirement(requirements);
}

void GotoGlobalSetpointType::update(
  const Eigen::Vector3d & global_position,
  const std::optional<float> & heading,
  const std::optional<float> & max_horizontal_speed,
  const std::optional<float> & max_vertical_speed,
  const std::optional<float> & max_heading_rate)
{
  if (!_map_projection->isInitialized()) {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(),
      "Goto global setpoint update failed: map projection is uninitialized. Is fmu/out/vehicle_local_position published?");
    return;
  }

  Eigen::Vector3f local_position = _map_projection->globalToLocal(global_position);
  _goto_setpoint->update(
    local_position, heading, max_horizontal_speed, max_vertical_speed,
    max_heading_rate);
}

} // namespace px4_ros2
