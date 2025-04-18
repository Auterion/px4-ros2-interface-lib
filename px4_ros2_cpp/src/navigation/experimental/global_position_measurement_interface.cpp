/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/navigation/experimental/global_position_measurement_interface.hpp>
#include <px4_ros2/utils/message_version.hpp>

using Eigen::Vector2d;
using px4_msgs::msg::VehicleGlobalPosition;

namespace px4_ros2
{

GlobalPositionMeasurementInterface::GlobalPositionMeasurementInterface(rclcpp::Node & node)
: PositionMeasurementInterfaceBase(node)
{
  _aux_global_position_pub =
    node.create_publisher<VehicleGlobalPosition>(
    topicNamespacePrefix() + "fmu/in/aux_global_position" +
    px4_ros2::getMessageNameVersion<VehicleGlobalPosition>(),
    10);
}

void GlobalPositionMeasurementInterface::update(
  const GlobalPositionMeasurement & global_position_measurement) const
{
  // Run basic sanity checks on global position measurement
  if (!isMeasurementNonEmpty(global_position_measurement)) {
    throw NavigationInterfaceInvalidArgument("Measurement values are all empty.");
  }

  if (!isVarianceValid(global_position_measurement)) {
    throw NavigationInterfaceInvalidArgument("Measurement has an invalid variance value.");
  }

  if (!isValueNotNAN(global_position_measurement)) {
    throw NavigationInterfaceInvalidArgument("Measurement value contains a NAN.");
  }

  if (global_position_measurement.timestamp_sample.nanoseconds() == 0) {
    throw NavigationInterfaceInvalidArgument("Measurement timestamp sample is empty.");
  }

  // Populate aux global position
  VehicleGlobalPosition aux_global_position;
  aux_global_position.lat_lon_reset_counter = _lat_lon_reset_counter;

  aux_global_position.timestamp_sample =
    global_position_measurement.timestamp_sample.nanoseconds() *
    1e-3;

  // Lat lon
  const Vector2d lat_lon = global_position_measurement.lat_lon.value_or(
    Vector2d{NAN,
      NAN});
  aux_global_position.lat = lat_lon[0];
  aux_global_position.lon = lat_lon[1];
  aux_global_position.eph =
    sqrt(
    global_position_measurement.horizontal_variance.value_or(
      NAN));

  // Altitude
  aux_global_position.alt = global_position_measurement.altitude_msl.value_or(NAN);
  aux_global_position.epv = sqrt(global_position_measurement.vertical_variance.value_or(NAN));

  // Publish
  aux_global_position.timestamp = 0; // Let PX4 set the timestamp
  _aux_global_position_pub->publish(aux_global_position);
}

bool GlobalPositionMeasurementInterface::isMeasurementNonEmpty(
  const GlobalPositionMeasurement & measurement) const
{
  return measurement.lat_lon.has_value() || measurement.altitude_msl.has_value();
}

bool GlobalPositionMeasurementInterface::isVarianceValid(
  const GlobalPositionMeasurement & measurement)
const
{
  if (measurement.lat_lon.has_value() &&
    (!measurement.horizontal_variance.has_value() || measurement.horizontal_variance.value() <= 0))
  {
    RCLCPP_ERROR_ONCE(_node.get_logger(), "Measurement lat_lon has an invalid variance value.");
    return false;
  }
  if (measurement.altitude_msl.has_value() &&
    (!measurement.vertical_variance.has_value() || measurement.vertical_variance.value() <= 0))
  {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(),
      "Measurement altitude_msl has an invalid variance value.");
    return false;
  }

  return true;
}

bool GlobalPositionMeasurementInterface::isFrameValid(const GlobalPositionMeasurement & measurement)
const
{
  return true;
}

bool GlobalPositionMeasurementInterface::isValueNotNAN(
  const GlobalPositionMeasurement & measurement)
const
{
  if (measurement.lat_lon.has_value() && measurement.lat_lon.value().hasNaN()) {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(),
      "Measurement value lat_lon is defined but contains a NAN.");
    return false;
  }
  if (measurement.horizontal_variance.has_value() &&
    std::isnan(measurement.horizontal_variance.value()))
  {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(), "Measurement value horizontal_variance is defined but contains a NAN.");
    return false;
  }
  if (measurement.altitude_msl.has_value() && std::isnan(measurement.altitude_msl.value())) {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(),
      "Measurement value altitude_msl is defined but contains a NAN.");
    return false;
  }
  if (measurement.vertical_variance.has_value() &&
    std::isnan(measurement.vertical_variance.value()))
  {
    RCLCPP_ERROR_ONCE(
      _node.get_logger(), "Measurement value vertical_variance is defined but contains a NAN.");
    return false;
  }
  return true;
}

} // namespace px4_ros2
