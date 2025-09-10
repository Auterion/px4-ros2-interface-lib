/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/mission/trajectory/trajectory_executor.hpp>
#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>

/** \ingroup mission_multicopter
 *  @{
 */

namespace px4_ros2::multicopter
{
/**
 * @brief Trajectory executor using goto setpoints
 * @ingroup mission_multicopter
 */
class WaypointTrajectoryExecutor : public TrajectoryExecutorInterface
{
public:
  explicit WaypointTrajectoryExecutor(ModeBase & mode, float acceptance_radius = 2.0f);

  ~WaypointTrajectoryExecutor() override = default;

  bool navigationItemTypeSupported(NavigationItemType type) override;
  bool frameSupported(MissionFrame frame) override;
  void runTrajectory(const TrajectoryConfig & config) override;
  void updateSetpoint() override;

private:
  void continueNextItem();
  bool positionReached(const Eigen::Vector3d & target_position_m, float acceptance_radius) const;
  bool headingReached(float target_heading_rad) const;

  TrajectoryConfig _current_trajectory;
  const float _acceptance_radius;
  std::shared_ptr<OdometryGlobalPosition> _vehicle_global_position;
  std::shared_ptr<OdometryAttitude> _vehicle_attitude;
  std::shared_ptr<MulticopterGotoGlobalSetpointType> _setpoint;
  std::optional<int> _current_index;
  rclcpp::Node & _node;
};

} // namespace px4_ros2::multicopter
/** @}*/
