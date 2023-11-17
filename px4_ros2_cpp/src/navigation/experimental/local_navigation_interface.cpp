/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/local_navigation_interface.hpp>

namespace px4_ros2
{

LocalNavigationInterface::LocalNavigationInterface(
  Context & context, uint8_t pose_frame,
  uint8_t velocity_frame)
: _node(context.node()),
  _pose_frame(pose_frame),
  _velocity_frame(velocity_frame)
{
  _aux_local_position_pub =
    context.node().create_publisher<AuxLocalPosition>(
    context.topicNamespacePrefix() + AUX_LOCAL_POSITION_TOPIC, 10);

}

void LocalNavigationInterface::update(LocalPositionEstimate & local_position_estimate)
{
  AuxLocalPosition aux_local_position;

  aux_local_position.timestamp_sample = local_position_estimate.timestamp_sample;

  // Position
  aux_local_position.pose_frame = _pose_frame;

  const Vector2f position_xy = local_position_estimate.position_xy.value_or(Vector2f{NAN, NAN});
  aux_local_position.position[0] = position_xy[0];
  aux_local_position.position[1] = position_xy[1];
  aux_local_position.position[2] = local_position_estimate.position_z.value_or(NAN);

  const Vector2f position_xy_variance = local_position_estimate.position_xy_variance.value_or(
    Vector2f{NAN, NAN});
  aux_local_position.position_variance[0] = position_xy_variance[0];
  aux_local_position.position_variance[1] = position_xy_variance[1];
  aux_local_position.position_variance[2] =
    local_position_estimate.position_z_variance.value_or(NAN);

  // Attitude
  const Quaternionf attitude_quaternion = local_position_estimate.attitude_quaternion.value_or(
    Quaternionf{NAN, NAN, NAN, NAN});
  aux_local_position.q[0] = attitude_quaternion.w();
  aux_local_position.q[1] = attitude_quaternion.x();
  aux_local_position.q[2] = attitude_quaternion.y();
  aux_local_position.q[3] = attitude_quaternion.z();

  const Vector3f attitude_variance = local_position_estimate.attitude_variance.value_or(
    Vector3f{NAN, NAN, NAN});
  aux_local_position.orientation_variance[0] = attitude_variance[0];
  aux_local_position.orientation_variance[1] = attitude_variance[1];
  aux_local_position.orientation_variance[2] = attitude_variance[2];

  // Velocity
  aux_local_position.velocity_frame = _velocity_frame;

  const Vector2f velocity_xy = local_position_estimate.velocity_xy.value_or(Vector2f{NAN, NAN});
  aux_local_position.velocity[0] = velocity_xy[0];
  aux_local_position.velocity[1] = velocity_xy[1];
  aux_local_position.velocity[2] = local_position_estimate.velocity_z.value_or(NAN);

  const Vector2f velocity_xy_variance = local_position_estimate.velocity_xy_variance.value_or(
    Vector2f{NAN, NAN});
  aux_local_position.velocity_variance[0] = velocity_xy_variance[0];
  aux_local_position.velocity_variance[1] = velocity_xy_variance[1];
  aux_local_position.velocity_variance[2] =
    local_position_estimate.velocity_z_variance.value_or(NAN);

  // Publish
  aux_local_position.timestamp = _node.get_clock()->now().nanoseconds() * 1e-3;
  _aux_local_position_pub->publish(aux_local_position);

  std::cout << "Published aux local position!\n";
}

} // namespace px4_ros2
