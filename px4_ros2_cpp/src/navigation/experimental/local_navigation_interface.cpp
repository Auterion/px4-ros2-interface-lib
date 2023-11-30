/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/local_navigation_interface.hpp>

using Eigen::Vector2f, Eigen::Quaternionf, Eigen::Vector3f;

namespace px4_ros2
{

LocalNavigationInterface::LocalNavigationInterface(
  rclcpp::Node & node, const PoseFrame pose_frame,
  const VelocityFrame velocity_frame)
: NavigationInterfaceBase(node)
{
  _pose_frame = poseFrameToMessageFrame(pose_frame);
  _velocity_frame = velocityFrameToMessageFrame(velocity_frame);

  _aux_local_position_pub = node.create_publisher<AuxLocalPosition>(
    topicNamespacePrefix() + "/fmu/in/vehicle_visual_odometry", 10);
}

void LocalNavigationInterface::update(
  const LocalPositionEstimate & local_position_estimate) const
{
  // Run basic sanity checks on local position estimate
  if (!isEstimateNonEmpty(local_position_estimate)) {
    throw NavigationInterfaceInvalidArgument("Estimate values are all empty.");
  }

  if (!isVarianceValid(local_position_estimate)) {
    throw NavigationInterfaceInvalidArgument("Estimate has an invalid variance value.");
  }

  if (!isFrameValid(local_position_estimate)) {
    throw NavigationInterfaceInvalidArgument(
            "Estimate has an unknown reference frame.");
  }

  if (!isValueNotNAN(local_position_estimate)) {
    throw NavigationInterfaceInvalidArgument("Estimate value contains a NAN.");
  }

  if (local_position_estimate.timestamp_sample.nanoseconds() == 0) {
    throw NavigationInterfaceInvalidArgument("Estimate timestamp sample is empty.");
  }

  // Populate aux local position message
  AuxLocalPosition aux_local_position;

  aux_local_position.timestamp_sample = local_position_estimate.timestamp_sample.nanoseconds() *
    1e-3;

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

  // Angular velocity (unused at the moment)
  aux_local_position.angular_velocity = {NAN, NAN, NAN};

  // Publish
  aux_local_position.timestamp = _node.get_clock()->now().nanoseconds() * 1e-3;
  _aux_local_position_pub->publish(aux_local_position);
}

bool LocalNavigationInterface::isEstimateNonEmpty(const LocalPositionEstimate & estimate) const
{
  return estimate.position_xy.has_value() ||
         estimate.position_z.has_value() ||
         estimate.velocity_xy.has_value() ||
         estimate.velocity_z.has_value() ||
         estimate.attitude_quaternion.has_value();
}

bool LocalNavigationInterface::isVarianceValid(const LocalPositionEstimate & estimate) const
{
  if (estimate.position_xy.has_value() &&
    (!estimate.position_xy_variance.has_value() ||
    (estimate.position_xy_variance.value().array() <= 0).any()))
  {
    RCLCPP_ERROR(_node.get_logger(), "Estimate value position_xy has an invalid variance value.");
    return false;
  }

  if (estimate.position_z.has_value() &&
    (!estimate.position_z_variance.has_value() || estimate.position_z_variance <= 0))
  {
    RCLCPP_ERROR(_node.get_logger(), "Estimate value position_z has an invalid variance value.");
    return false;
  }

  if (estimate.velocity_xy.has_value() &&
    (!estimate.velocity_xy_variance.has_value() ||
    (estimate.velocity_xy_variance.value().array() <= 0).any()))
  {
    RCLCPP_ERROR(_node.get_logger(), "Estimate value velocity_xy has an invalid variance value.");
    return false;
  }

  if (estimate.velocity_z.has_value() &&
    (!estimate.velocity_z_variance.has_value() || estimate.velocity_z_variance <= 0))
  {
    RCLCPP_ERROR(_node.get_logger(), "Estimate value velocity_z has an invalid variance value.");
    return false;
  }

  if (estimate.attitude_quaternion.has_value() &&
    (!estimate.attitude_variance.has_value() ||
    (estimate.attitude_variance.value().array() <= 0).any()))
  {
    RCLCPP_ERROR(
      _node.get_logger(),
      "Estimate value attitude_quaternion has an invalid variance value.");
    return false;
  }

  return true;
}

bool LocalNavigationInterface::isFrameValid(const LocalPositionEstimate & estimate) const
{
  if ((estimate.position_xy.has_value() || estimate.position_z.has_value()) &&
    _pose_frame == AuxLocalPosition::POSE_FRAME_UNKNOWN)
  {
    RCLCPP_ERROR(
      _node.get_logger(),
      "Position estimate has unknown pose frame.");
    return false;
  }

  if ((estimate.velocity_xy.has_value() || estimate.velocity_z.has_value()) &&
    _pose_frame == AuxLocalPosition::VELOCITY_FRAME_UNKNOWN)
  {
    RCLCPP_ERROR(
      _node.get_logger(),
      "Velocity estimate has unknown velocity frame.");
    return false;
  }

  return true;
}

bool LocalNavigationInterface::isValueNotNAN(const LocalPositionEstimate & estimate) const
{
  if (estimate.position_xy.has_value() && estimate.position_xy.value().hasNaN()) {
    RCLCPP_ERROR(_node.get_logger(), "Estimate value position_xy is defined but contains a NAN.");
    return false;
  }
  if (estimate.position_xy_variance.has_value() &&
    estimate.position_xy_variance.value().hasNaN())
  {
    RCLCPP_ERROR(
      _node.get_logger(), "Estimate value position_xy_variance is defined but contains a NAN.");
    return false;
  }
  if (estimate.position_z.has_value() && std::isnan(estimate.position_z.value())) {
    RCLCPP_ERROR(_node.get_logger(), "Estimate value position_z is defined but contains a NAN.");
    return false;
  }
  if (estimate.position_z_variance.has_value() &&
    std::isnan(estimate.position_z_variance.value()))
  {
    RCLCPP_ERROR(
      _node.get_logger(), "Estimate value position_z_variance is defined but contains a NAN.");
    return false;
  }
  if (estimate.velocity_xy.has_value() && estimate.velocity_xy.value().hasNaN()) {
    RCLCPP_ERROR(_node.get_logger(), "Estimate value velocity_xy is defined but contains a NAN.");
    return false;
  }
  if (estimate.velocity_xy_variance.has_value() &&
    estimate.velocity_xy_variance.value().hasNaN())
  {
    RCLCPP_ERROR(
      _node.get_logger(), "Estimate value velocity_xy_variance is defined but contains a NAN.");
    return false;
  }
  if (estimate.velocity_z.has_value() && std::isnan(estimate.velocity_z.value())) {
    RCLCPP_ERROR(_node.get_logger(), "Estimate value velocity_z is defined but contains a NAN.");
    return false;
  }
  if (estimate.velocity_z_variance.has_value() &&
    std::isnan(estimate.velocity_z_variance.value()))
  {
    RCLCPP_ERROR(
      _node.get_logger(), "Estimate value velocity_z_variance is defined but contains a NAN.");
    return false;
  }
  if (estimate.attitude_quaternion.has_value() &&
    estimate.attitude_quaternion.value().coeffs().hasNaN())
  {
    RCLCPP_ERROR(
      _node.get_logger(), "Estimate value attitude_quaternion is defined but contains a NAN.");
    return false;
  }
  if (estimate.attitude_variance.has_value() && estimate.attitude_variance.value().hasNaN()) {
    RCLCPP_ERROR(
      _node.get_logger(), "Estimate value attitude_variance is defined but contains a NAN.");
    return false;
  }
  return true;
}


} // namespace px4_ros2
