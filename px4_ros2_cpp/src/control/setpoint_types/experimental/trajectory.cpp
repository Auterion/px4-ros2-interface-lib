/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>


namespace px4_ros2
{

TrajectorySetpointType::TrajectorySetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _trajectory_setpoint_pub = context.node().create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    context.topicNamespacePrefix() + "fmu/in/trajectory_setpoint", 1);
}

void TrajectorySetpointType::update(
  const Eigen::Vector3f & velocity_ned_m_s,
  const std::optional<Eigen::Vector3f> & acceleration_ned_m_s2,
  std::optional<float> yaw_ned_rad,
  std::optional<float> yaw_rate_ned_rad_s)
{
  onUpdate();

  px4_msgs::msg::TrajectorySetpoint sp{};
  sp.timestamp = _node.get_clock()->now().nanoseconds() / 1000;

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

void TrajectorySetpointType::updatePosition(
  const Eigen::Vector3f & position_ned_m)
{
  onUpdate();

  px4_msgs::msg::TrajectorySetpoint sp{};
  sp.timestamp = _node.get_clock()->now().nanoseconds() / 1000;

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
  config.velocity_enabled = true;
  config.position_enabled = true;
  config.altitude_enabled = true;
  config.climb_rate_enabled = true;
  return config;
}
} // namespace px4_ros2
