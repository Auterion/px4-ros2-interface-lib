/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/mission/trajectory/multicopter/waypoint_trajectory_executor.hpp>

using namespace px4_ros2::literals; // NOLINT

namespace px4_ros2::multicopter
{
WaypointTrajectoryExecutor::WaypointTrajectoryExecutor(
  ModeBase & mode,
  float acceptance_radius)
: _acceptance_radius(acceptance_radius), _node(mode.node())
{
  _setpoint = std::make_shared<MulticopterGotoGlobalSetpointType>(mode);
  _vehicle_global_position = std::make_shared<OdometryGlobalPosition>(mode);
  _vehicle_attitude = std::make_shared<OdometryAttitude>(mode);
}

bool WaypointTrajectoryExecutor::navigationItemTypeSupported(NavigationItemType type)
{
  return type == NavigationItemType::Waypoint;
}

bool WaypointTrajectoryExecutor::frameSupported(MissionFrame frame)
{
  return frame == MissionFrame::Global;
}

void WaypointTrajectoryExecutor::runTrajectory(const TrajectoryConfig & config)
{
  _current_trajectory = config;
  _current_index = config.start_index;
}

void WaypointTrajectoryExecutor::updateSetpoint()
{
  if (!_current_index) {
    return;
  }
  const auto * navigation_item = std::get_if<NavigationItem>(
    &_current_trajectory.trajectory->items()[*_current_index]);
  if (!navigation_item) {
    // Not expected to happen
    continueNextItem();
    return;
  }

  if (!_vehicle_global_position->positionValid()) {
    RCLCPP_ERROR(_node.get_logger(), "Global position not valid, aborting");
    _current_trajectory.on_failure();
    return;
  }

  const auto & waypoint = std::get<Waypoint>(navigation_item->data);
  const Eigen::Vector3d & target_position = waypoint.coordinate;
  std::optional<float> heading_target_rad{};

  if (horizontalDistanceToGlobalPosition(
      _vehicle_global_position->position(),
      target_position) > 0.1f)
  {
    // Stop caring about heading when the arctangent becomes undefined
    heading_target_rad = headingToGlobalPosition(
      _vehicle_global_position->position(), target_position);
  }

  const auto & options = _current_trajectory.options;
  _setpoint->update(
    target_position, heading_target_rad, options.horizontal_velocity,
    options.vertical_velocity, options.max_heading_rate);

  float acceptance_radius = _acceptance_radius;
  if (*_current_index == _current_trajectory.end_index && _current_trajectory.stop_at_last) {
    acceptance_radius /= 2.f;
  }

  if (positionReached(target_position, acceptance_radius)) {
    continueNextItem();
  }
}

void WaypointTrajectoryExecutor::continueNextItem()
{
  const int index_reached = *_current_index;
  _current_index = *_current_index + 1;
  if (*_current_index > _current_trajectory.end_index) {
    _current_index.reset();
  }
  // Call the callback after updating the state (as it might set a new trajectory already)
  _current_trajectory.on_index_reached(index_reached);
}

bool WaypointTrajectoryExecutor::positionReached(
  const Eigen::Vector3d & target_position_m,
  float acceptance_radius) const
{
  const float position_error = distanceToGlobalPosition(
    _vehicle_global_position->position(), target_position_m);
  return position_error < acceptance_radius;
}

bool WaypointTrajectoryExecutor::headingReached(float target_heading_rad) const
{
  static constexpr float kHeadingErrorThreshold = 7.0_deg;
  const float heading_error_wrapped = wrapPi(
    target_heading_rad - _vehicle_attitude->yaw());
  return fabsf(heading_error_wrapped) < kHeadingErrorThreshold;
}

} // namespace px4_ros2::multicopter
