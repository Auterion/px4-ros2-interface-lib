/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/trajectory.hpp>


namespace px4_ros2
{

TrajectorySetpointType::TrajectorySetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _trajectory_setpoint_pub = context.node().create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    context.topicNamespacePrefix() + "/fmu/in/trajectory_setpoint", 1);
}

void TrajectorySetpointType::update(const Eigen::Vector3f & velocity_setpoint_ned_m_s)
{
  onUpdate();

  px4_msgs::msg::TrajectorySetpoint sp{};
  // TODO
  sp.yaw = NAN;
  sp.yawspeed = NAN;
  sp.position[0] = sp.position[1] = sp.position[2] = NAN;
  sp.acceleration[0] = sp.acceleration[1] = sp.acceleration[2] = NAN;
  sp.velocity[0] = velocity_setpoint_ned_m_s(0);
  sp.velocity[1] = velocity_setpoint_ned_m_s(1);
  sp.velocity[2] = velocity_setpoint_ned_m_s(2);
  sp.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
  _trajectory_setpoint_pub->publish(sp);

}

SetpointBase::Configuration TrajectorySetpointType::getConfiguration()
{
  Configuration config{};
  return config;
}
} // namespace px4_ros2
