/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/navigation/experimental/global_navigation_interface.hpp>


namespace px4_ros2
{

GlobalNavigationInterface::GlobalNavigationInterface(rclcpp::Node & node)
: _node(node)
{
  _aux_global_position_pub =
    node.create_publisher<AuxGlobalPosition>(AUX_GLOBAL_POSITION_TOPIC, 10);
}

NavigationInterfaceReturnCode GlobalNavigationInterface::update(
  const GlobalPositionEstimate & global_position_estimate) const
{
  // Run basic sanity checks on global position estimate
  if (_checkEstimateEmpty(global_position_estimate)) {
    RCLCPP_DEBUG(_node.get_logger(), "Estimate values are all empty.");
    return NavigationInterfaceReturnCode::ESTIMATE_EMPTY;
  }

  if (!_checkVarianceValid(global_position_estimate)) {
    return NavigationInterfaceReturnCode::ESTIMATE_VARIANCE_INVALID;
  }

  if (!_checkValuesNotNAN(global_position_estimate)) {
    return NavigationInterfaceReturnCode::ESTIMATE_VALUE_NAN;
  }

  if (global_position_estimate.timestamp_sample == 0) {
    RCLCPP_DEBUG(_node.get_logger(), "Estimate timestamp sample is empty.");
    return NavigationInterfaceReturnCode::ESTIMATE_MISSING_TIMESTAMP;
  }

  // Populate aux global position
  px4_msgs::msg::AuxGlobalPosition aux_global_position;

  aux_global_position.timestamp_sample = global_position_estimate.timestamp_sample;

  // Lat lon
  const Vector2f lat_lon = global_position_estimate.lat_lon.value_or(
    Vector2f{NAN,
      NAN});
  aux_global_position.latitude = lat_lon[0];
  aux_global_position.longitude = lat_lon[1];
  aux_global_position.positional_uncertainty =
    global_position_estimate.positional_uncertainty.value_or(
    NAN);

  // Altitude
  aux_global_position.altitude_agl = global_position_estimate.altitude_agl.value_or(NAN);

  // Valid flag
  aux_global_position.valid = true;

  // Publish
  aux_global_position.timestamp = _node.get_clock()->now().nanoseconds() * 1e-3;
  _aux_global_position_pub->publish(aux_global_position);

  return NavigationInterfaceReturnCode::SUCCESS;
}

bool GlobalNavigationInterface::_checkEstimateEmpty(const GlobalPositionEstimate & estimate) const
{
  return !estimate.lat_lon.has_value() && !estimate.altitude_agl.has_value();
}

bool GlobalNavigationInterface::_checkVarianceValid(const GlobalPositionEstimate & estimate) const
{
  if ((estimate.lat_lon.has_value() || estimate.altitude_agl.has_value()) &&
    (!estimate.positional_uncertainty.has_value() || estimate.positional_uncertainty.value() <= 0))
  {
    RCLCPP_DEBUG(
      _node.get_logger(),
      "Estimate value lat_lon has an invalid variance value.");
    return false;
  }

  return true;
}

bool GlobalNavigationInterface::_checkFrameValid(const GlobalPositionEstimate & estimate) const
{
  return true;
}

bool GlobalNavigationInterface::_checkValuesNotNAN(const GlobalPositionEstimate & estimate) const
{
  if (estimate.lat_lon.has_value() && estimate.lat_lon.value().hasNaN()) {
    RCLCPP_DEBUG(
      _node.get_logger(),
      "Estimate value lat_lon is defined but contains a NAN.");
    return false;
  }
  if (estimate.altitude_agl.has_value() && estimate.altitude_agl == NAN) {
    RCLCPP_DEBUG(
      _node.get_logger(),
      "Estimate value altitude_agl is defined but contains a NAN.");
    return false;
  }
  if (estimate.positional_uncertainty.has_value() && estimate.positional_uncertainty == NAN) {
    RCLCPP_DEBUG(
      _node.get_logger(),
      "Estimate value positional_uncertainty is defined but contains a NAN.");
    return false;
  }
  return true;
}

} // namespace px4_ros2
