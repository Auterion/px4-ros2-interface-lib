/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/fixedwing/path_setpoint.hpp>
#include <px4_ros2/common/requirement_flags.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

// ---------------------------------------------------------------------------
// FwPathSetpointType (local NED, SetpointBase)
// ---------------------------------------------------------------------------

FwPathSetpointType::FwPathSetpointType(Context & context)
: SetpointBase(context)
{
  _pub = context.node().create_publisher<px4_msgs::msg::PathSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/path_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::PathSetpoint>(),
    1);
}

void FwPathSetpointType::update(
  const Eigen::Vector3f & position_ned,
  const std::optional<Eigen::Vector3f> & tangent_ned,
  const std::optional<float> & height_rate,
  const std::optional<float> & curvature,
  const std::optional<float> & equivalent_airspeed)
{
  onUpdate();

  px4_msgs::msg::PathSetpoint sp{};
  sp.timestamp = 0;

  sp.position[0] = position_ned(0);
  sp.position[1] = position_ned(1);
  sp.position[2] = position_ned(2);

  if (tangent_ned.has_value()) {
    sp.tangent[0] = (*tangent_ned)(0);
    sp.tangent[1] = (*tangent_ned)(1);
    sp.tangent[2] = (*tangent_ned)(2);
  } else {
    sp.tangent[0] = sp.tangent[1] = sp.tangent[2] = NAN;
  }

  sp.height_rate = height_rate.value_or(NAN);
  sp.curvature = curvature.value_or(NAN);
  sp.equivalent_airspeed = equivalent_airspeed.value_or(NAN);

  _pub->publish(sp);
}

SetpointBase::Configuration FwPathSetpointType::getConfiguration()
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

// ---------------------------------------------------------------------------
// FwGlobalPathSetpointType (global lat/lon/alt, wraps FwPathSetpointType)
// ---------------------------------------------------------------------------

FwGlobalPathSetpointType::FwGlobalPathSetpointType(Context & context)
: _node(context.node()),
  _map_projection(std::make_unique<MapProjection>(context)),
  _path_setpoint(std::make_shared<FwPathSetpointType>(context))
{
  RequirementFlags requirements{};
  requirements.global_position = true;
  context.setRequirement(requirements);
}

void FwGlobalPathSetpointType::update(
  const Eigen::Vector3d & global_position,
  const std::optional<Eigen::Vector3f> & tangent_ned,
  const std::optional<float> & height_rate,
  const std::optional<float> & curvature,
  const std::optional<float> & equivalent_airspeed)
{
  if (!_map_projection->isInitialized()) {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(),
      "FW global path setpoint update failed: map projection is uninitialized. "
      "Is fmu/out/vehicle_local_position published?");
    return;
  }

  const Eigen::Vector3f local_position = _map_projection->globalToLocal(global_position);
  _path_setpoint->update(local_position, tangent_ned, height_rate, curvature, equivalent_airspeed);
}

} // namespace px4_ros2
