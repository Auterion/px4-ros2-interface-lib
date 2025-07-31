/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

TrajectorySetpointType::TrajectorySetpointType(Context & context, bool local_position_is_optional)
: SetpointBase(context), _node(context.node()), _local_position_is_optional(
    local_position_is_optional)
{
  _trajectory_setpoint_pub = context.node().create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    context.topicNamespacePrefix() + "fmu/in/trajectory_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::TrajectorySetpoint>(),
    1);
}

void TrajectorySetpointType::update(
  const Eigen::Vector3f & velocity_ned_m_s,
  const std::optional<Eigen::Vector3f> & acceleration_ned_m_s2,
  std::optional<float> yaw_ned_rad,
  std::optional<float> yaw_rate_ned_rad_s)
{
  onUpdate();

  px4_msgs::msg::TrajectorySetpoint sp{};
  sp.timestamp = 0; // Let PX4 set the timestamp

  sp.position[0] = sp.position[1] = sp.position[2] = NAN;
  sp.velocity[0] = velocity_ned_m_s.x();
  sp.velocity[1] = velocity_ned_m_s.y();
  sp.velocity[2] = velocity_ned_m_s.z();
  Eigen::Vector3f acceleration = acceleration_ned_m_s2.value_or(Eigen::Vector3f{NAN, NAN, NAN});
  sp.acceleration[0] = acceleration.x();
  sp.acceleration[1] = acceleration.y();
  sp.acceleration[2] = acceleration.z();
  sp.yaw = yaw_ned_rad.value_or(NAN);
  sp.yawspeed = yaw_rate_ned_rad_s.value_or(NAN);

  _trajectory_setpoint_pub->publish(sp);
}

void TrajectorySetpointType::update(const TrajectorySetpoint & setpoint)
{
  onUpdate();

  px4_msgs::msg::TrajectorySetpoint sp{};
  sp.timestamp = 0; // Let PX4 set the timestamp

  sp.position[0] = setpoint.position_ned_m_x.value_or(NAN);
  sp.position[1] = setpoint.position_ned_m_y.value_or(NAN);
  sp.position[2] = setpoint.position_ned_m_z.value_or(NAN);
  sp.velocity[0] = setpoint.velocity_ned_m_s_x.value_or(NAN);
  sp.velocity[1] = setpoint.velocity_ned_m_s_y.value_or(NAN);
  sp.velocity[2] = setpoint.velocity_ned_m_s_z.value_or(NAN);
  sp.acceleration[0] = setpoint.acceleration_ned_m_s2_x.value_or(NAN);
  sp.acceleration[1] = setpoint.acceleration_ned_m_s2_y.value_or(NAN);
  sp.acceleration[2] = setpoint.acceleration_ned_m_s2_z.value_or(NAN);
  sp.yaw = setpoint.yaw_ned_rad.value_or(NAN);
  sp.yawspeed = setpoint.yaw_rate_ned_rad_s.value_or(NAN);

  _trajectory_setpoint_pub->publish(sp);
}

void TrajectorySetpointType::updatePosition(
  const Eigen::Vector3f & position_ned_m)
{
  onUpdate();

  px4_msgs::msg::TrajectorySetpoint sp{};
  sp.timestamp = 0; // Let PX4 set the timestamp

  sp.position[0] = position_ned_m.x();
  sp.position[1] = position_ned_m.y();
  sp.position[2] = position_ned_m.z();
  sp.velocity[0] = sp.velocity[1] = sp.velocity[2] = NAN;
  sp.acceleration[0] = sp.acceleration[1] = sp.acceleration[2] = NAN;
  sp.yaw = NAN;
  sp.yawspeed = NAN;

  _trajectory_setpoint_pub->publish(sp);
}

SetpointBase::Configuration TrajectorySetpointType::getConfiguration()
{
  Configuration config{};
  config.rates_enabled = true;
  config.attitude_enabled = true;
  config.acceleration_enabled = true;
  config.position_enabled = true;
  config.velocity_enabled = true;
  config.altitude_enabled = true;
  config.climb_rate_enabled = true;
  config.local_position_is_optional = _local_position_is_optional;
  return config;
}
} // namespace px4_ros2
