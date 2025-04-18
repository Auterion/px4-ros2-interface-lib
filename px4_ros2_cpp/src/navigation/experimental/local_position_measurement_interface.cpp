/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>
#include <px4_ros2/utils/message_version.hpp>

using Eigen::Vector2f, Eigen::Quaternionf, Eigen::Vector3f;

namespace px4_ros2
{

LocalPositionMeasurementInterface::LocalPositionMeasurementInterface(
  rclcpp::Node & node, const PoseFrame pose_frame,
  const VelocityFrame velocity_frame)
: PositionMeasurementInterfaceBase(node),
  _pose_frame(poseFrameToMessageFrame(pose_frame)),
  _velocity_frame(velocityFrameToMessageFrame(velocity_frame))
{
  _aux_local_position_pub = node.create_publisher<AuxLocalPosition>(
    topicNamespacePrefix() + "fmu/in/vehicle_visual_odometry" +
    px4_ros2::getMessageNameVersion<AuxLocalPosition>(),
    10);
}

void LocalPositionMeasurementInterface::update(
  const LocalPositionMeasurement & local_position_measurement) const
{
  // Run basic sanity checks on local position measurement
  if (!isMeasurementNonEmpty(local_position_measurement)) {
    throw NavigationInterfaceInvalidArgument("Measurement values are all empty.");
  }

  if (!isVarianceValid(local_position_measurement)) {
    throw NavigationInterfaceInvalidArgument("Measurement has an invalid variance value.");
  }

  if (!isFrameValid(local_position_measurement)) {
    throw NavigationInterfaceInvalidArgument(
            "Measurement has an unknown reference frame.");
  }

  if (!isValueNotNAN(local_position_measurement)) {
    throw NavigationInterfaceInvalidArgument("Measurement value contains a NAN.");
  }

  if (local_position_measurement.timestamp_sample.nanoseconds() == 0) {
    throw NavigationInterfaceInvalidArgument("Measurement timestamp sample is empty.");
  }

  // Populate aux local position message
  AuxLocalPosition aux_local_position;

  aux_local_position.timestamp_sample = local_position_measurement.timestamp_sample.nanoseconds() *
    1e-3;

  // Position
  aux_local_position.pose_frame = _pose_frame;

  const Vector2f position_xy = local_position_measurement.position_xy.value_or(Vector2f{NAN, NAN});
  aux_local_position.position[0] = position_xy[0];
  aux_local_position.position[1] = position_xy[1];
  aux_local_position.position[2] = local_position_measurement.position_z.value_or(NAN);

  const Vector2f position_xy_variance = local_position_measurement.position_xy_variance.value_or(
    Vector2f{NAN, NAN});
  aux_local_position.position_variance[0] = position_xy_variance[0];
  aux_local_position.position_variance[1] = position_xy_variance[1];
  aux_local_position.position_variance[2] =
    local_position_measurement.position_z_variance.value_or(NAN);

  // Attitude
  const Quaternionf attitude_quaternion = local_position_measurement.attitude_quaternion.value_or(
    Quaternionf{NAN, NAN, NAN, NAN});
  aux_local_position.q[0] = attitude_quaternion.w();
  aux_local_position.q[1] = attitude_quaternion.x();
  aux_local_position.q[2] = attitude_quaternion.y();
  aux_local_position.q[3] = attitude_quaternion.z();

  const Vector3f attitude_variance = local_position_measurement.attitude_variance.value_or(
    Vector3f{NAN, NAN, NAN});
  aux_local_position.orientation_variance[0] = attitude_variance[0];
  aux_local_position.orientation_variance[1] = attitude_variance[1];
  aux_local_position.orientation_variance[2] = attitude_variance[2];

  // Velocity
  aux_local_position.velocity_frame = _velocity_frame;

  const Vector2f velocity_xy = local_position_measurement.velocity_xy.value_or(Vector2f{NAN, NAN});
  aux_local_position.velocity[0] = velocity_xy[0];
  aux_local_position.velocity[1] = velocity_xy[1];
  aux_local_position.velocity[2] = local_position_measurement.velocity_z.value_or(NAN);

  const Vector2f velocity_xy_variance = local_position_measurement.velocity_xy_variance.value_or(
    Vector2f{NAN, NAN});
  aux_local_position.velocity_variance[0] = velocity_xy_variance[0];
  aux_local_position.velocity_variance[1] = velocity_xy_variance[1];
  aux_local_position.velocity_variance[2] =
    local_position_measurement.velocity_z_variance.value_or(NAN);

  // Angular velocity (unused at the moment)
  aux_local_position.angular_velocity = {NAN, NAN, NAN};

  // Publish
  aux_local_position.timestamp = 0; // Let PX4 set the timestamp
  _aux_local_position_pub->publish(aux_local_position);
}

bool LocalPositionMeasurementInterface::isMeasurementNonEmpty(
  const LocalPositionMeasurement & measurement) const
{
  return measurement.position_xy.has_value() ||
         measurement.position_z.has_value() ||
         measurement.velocity_xy.has_value() ||
         measurement.velocity_z.has_value() ||
         measurement.attitude_quaternion.has_value();
}

bool LocalPositionMeasurementInterface::isVarianceValid(
  const LocalPositionMeasurement & measurement)
const
{
  if (measurement.position_xy.has_value() &&
    (!measurement.position_xy_variance.has_value() ||
    (measurement.position_xy_variance.value().array() <= 0).any()))
  {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(),
      "Measurement value position_xy has an invalid variance value.");
    return false;
  }

  if (measurement.position_z.has_value() &&
    (!measurement.position_z_variance.has_value() || measurement.position_z_variance <= 0))
  {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(), "Measurement value position_z has an invalid variance value.");
    return false;
  }

  if (measurement.velocity_xy.has_value() &&
    (!measurement.velocity_xy_variance.has_value() ||
    (measurement.velocity_xy_variance.value().array() <= 0).any()))
  {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(),
      "Measurement value velocity_xy has an invalid variance value.");
    return false;
  }

  if (measurement.velocity_z.has_value() &&
    (!measurement.velocity_z_variance.has_value() || measurement.velocity_z_variance <= 0))
  {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(), "Measurement value velocity_z has an invalid variance value.");
    return false;
  }

  if (measurement.attitude_quaternion.has_value() &&
    (!measurement.attitude_variance.has_value() ||
    (measurement.attitude_variance.value().array() <= 0).any()))
  {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(),
      "Measurement value attitude_quaternion has an invalid variance value.");
    return false;
  }

  return true;
}

bool LocalPositionMeasurementInterface::isFrameValid(const LocalPositionMeasurement & measurement)
const
{
  if ((measurement.position_xy.has_value() || measurement.position_z.has_value()) &&
    _pose_frame == AuxLocalPosition::POSE_FRAME_UNKNOWN)
  {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(),
      "Position measurement has unknown pose frame.");
    return false;
  }

  if ((measurement.velocity_xy.has_value() || measurement.velocity_z.has_value()) &&
    _velocity_frame == AuxLocalPosition::VELOCITY_FRAME_UNKNOWN)
  {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(),
      "Velocity measurement has unknown velocity frame.");
    return false;
  }

  return true;
}

bool LocalPositionMeasurementInterface::isValueNotNAN(const LocalPositionMeasurement & measurement)
const
{
  if (measurement.position_xy.has_value() && measurement.position_xy.value().hasNaN()) {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(),
      "Measurement value position_xy is defined but contains a NAN.");
    return false;
  }
  if (measurement.position_xy_variance.has_value() &&
    measurement.position_xy_variance.value().hasNaN())
  {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(), "Measurement value position_xy_variance is defined but contains a NAN.");
    return false;
  }
  if (measurement.position_z.has_value() && std::isnan(measurement.position_z.value())) {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(), "Measurement value position_z is defined but contains a NAN.");
    return false;
  }
  if (measurement.position_z_variance.has_value() &&
    std::isnan(measurement.position_z_variance.value()))
  {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(), "Measurement value position_z_variance is defined but contains a NAN.");
    return false;
  }
  if (measurement.velocity_xy.has_value() && measurement.velocity_xy.value().hasNaN()) {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(),
      "Measurement value velocity_xy is defined but contains a NAN.");
    return false;
  }
  if (measurement.velocity_xy_variance.has_value() &&
    measurement.velocity_xy_variance.value().hasNaN())
  {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(), "Measurement value velocity_xy_variance is defined but contains a NAN.");
    return false;
  }
  if (measurement.velocity_z.has_value() && std::isnan(measurement.velocity_z.value())) {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(), "Measurement value velocity_z is defined but contains a NAN.");
    return false;
  }
  if (measurement.velocity_z_variance.has_value() &&
    std::isnan(measurement.velocity_z_variance.value()))
  {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(), "Measurement value velocity_z_variance is defined but contains a NAN.");
    return false;
  }
  if (measurement.attitude_quaternion.has_value() &&
    measurement.attitude_quaternion.value().coeffs().hasNaN())
  {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(), "Measurement value attitude_quaternion is defined but contains a NAN.");
    return false;
  }
  if (measurement.attitude_variance.has_value() && measurement.attitude_variance.value().hasNaN()) {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(), "Measurement value attitude_variance is defined but contains a NAN.");
    return false;
  }
  return true;
}


} // namespace px4_ros2
