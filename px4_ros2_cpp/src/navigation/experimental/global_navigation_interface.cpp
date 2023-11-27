/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/navigation/experimental/global_navigation_interface.hpp>


namespace px4_ros2
{

GlobalNavigationInterface::GlobalNavigationInterface(rclcpp::Node & node)
: NavigationInterfaceBase(node)
{
  _aux_global_position_pub =
    node.create_publisher<VehicleGlobalPosition>(
    topicNamespacePrefix() + "/fmu/in/aux_global_position", 10);
}

NavigationInterfaceReturnCode GlobalNavigationInterface::update(
  const GlobalPositionEstimate & global_position_estimate) const
{
  // Run basic sanity checks on global position estimate
  if (!_isEstimateNonEmpty(global_position_estimate)) {
    RCLCPP_DEBUG(_node.get_logger(), "Estimate values are all empty.");
    return NavigationInterfaceReturnCode::ESTIMATE_EMPTY;
  }

  if (!_isVarianceValid(global_position_estimate)) {
    return NavigationInterfaceReturnCode::ESTIMATE_VARIANCE_INVALID;
  }

  if (!_isValueNotNAN(global_position_estimate)) {
    return NavigationInterfaceReturnCode::ESTIMATE_VALUE_NAN;
  }

  if (global_position_estimate.timestamp_sample.nanoseconds() == 0) {
    RCLCPP_DEBUG(_node.get_logger(), "Estimate timestamp sample is empty.");
    return NavigationInterfaceReturnCode::ESTIMATE_MISSING_TIMESTAMP;
  }

  // Populate aux global position
  px4_msgs::msg::VehicleGlobalPosition aux_global_position;

  aux_global_position.timestamp_sample = global_position_estimate.timestamp_sample.nanoseconds() *
    1e-3;

  // Lat lon
  const Vector2d lat_lon = global_position_estimate.lat_lon.value_or(
    Vector2d{NAN,
      NAN});
  aux_global_position.lat = lat_lon[0];
  aux_global_position.lon = lat_lon[1];
  aux_global_position.eph =
    sqrt(
    global_position_estimate.horizontal_variance.value_or(
      NAN));

  // Altitude
  aux_global_position.alt = global_position_estimate.altitude_msl.value_or(NAN);
  aux_global_position.epv = sqrt(global_position_estimate.vertical_variance.value_or(NAN));

  // Publish
  aux_global_position.timestamp = _node.get_clock()->now().nanoseconds() * 1e-3;
  _aux_global_position_pub->publish(aux_global_position);

  return NavigationInterfaceReturnCode::SUCCESS;
}

bool GlobalNavigationInterface::_isEstimateNonEmpty(const GlobalPositionEstimate & estimate) const
{
  return estimate.lat_lon.has_value() || estimate.altitude_msl.has_value();
}

bool GlobalNavigationInterface::_isVarianceValid(const GlobalPositionEstimate & estimate) const
{
  if (estimate.lat_lon.has_value() &&
    (!estimate.horizontal_variance.has_value() || estimate.horizontal_variance.value() <= 0))
  {
    RCLCPP_DEBUG(
      _node.get_logger(),
      "Estimate lat_lon has an invalid standard deviation value.");
    return false;
  }
  if (estimate.altitude_msl.has_value() &&
    (!estimate.vertical_variance.has_value() || estimate.vertical_variance.value() <= 0))
  {
    RCLCPP_DEBUG(
      _node.get_logger(),
      "Estimate altitude_msl has an invalid standard deviation value.");
    return false;
  }

  return true;
}

bool GlobalNavigationInterface::_isFrameValid(const GlobalPositionEstimate & estimate) const
{
  return true;
}

bool GlobalNavigationInterface::_isValueNotNAN(const GlobalPositionEstimate & estimate) const
{
  if (estimate.lat_lon.has_value() && estimate.lat_lon.value().hasNaN()) {
    RCLCPP_DEBUG(
      _node.get_logger(),
      "Estimate value lat_lon is defined but contains a NAN.");
    return false;
  }
  if (estimate.horizontal_variance.has_value() && estimate.horizontal_variance == NAN) {
    RCLCPP_DEBUG(
      _node.get_logger(),
      "Estimate value horizontal_variance is defined but contains a NAN.");
    return false;
  }
  if (estimate.altitude_msl.has_value() && estimate.altitude_msl == NAN) {
    RCLCPP_DEBUG(
      _node.get_logger(),
      "Estimate value altitude_msl is defined but contains a NAN.");
    return false;
  }
  if (estimate.vertical_variance.has_value() && estimate.vertical_variance == NAN) {
    RCLCPP_DEBUG(
      _node.get_logger(),
      "Estimate value vertical_variance is defined but contains a NAN.");
    return false;
  }
  return true;
}

} // namespace px4_ros2
