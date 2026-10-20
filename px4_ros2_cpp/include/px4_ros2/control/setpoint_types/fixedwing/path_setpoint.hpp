/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <optional>
#include <px4_msgs/msg/path_setpoint.hpp>
#include <px4_ros2/common/setpoint_base.hpp>
#include <px4_ros2/utils/geodesic.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types
 *  @{
 */

/**
 * @brief Setpoint type for fixed-wing path following in the local NED frame.
 *
 * Publishes a PathSetpoint to the PX4 guidance controller. The setpoint
 * describes a path in local NED coordinates with an optional tangent direction,
 * height rate, and curvature. Setting any field to NaN (by leaving it unset)
 * tells PX4 not to control that quantity.
 */
class FwPathSetpointType : public SetpointBase
{
public:
  explicit FwPathSetpointType(Context & context);
  ~FwPathSetpointType() override = default;

  Configuration getConfiguration() override;

  /**
   * @brief Update the path setpoint.
   *
   * @param position_ned         Target path position in local NED frame [m]
   * @param tangent_ned          Path tangent unit vector in NED frame; unset = not controlled
   * @param height_rate          Desired height rate [m/s]; unset = not controlled
   * @param curvature            Path curvature [1/m]; unset = not controlled
   * @param equivalent_airspeed  Target equivalent airspeed [m/s]; unset = not controlled
   */
  void update(
    const Eigen::Vector3f & position_ned,
    const std::optional<Eigen::Vector3f> & tangent_ned = {},
    const std::optional<float> & height_rate = {},
    const std::optional<float> & curvature = {},
    const std::optional<float> & equivalent_airspeed = {});

  float desiredUpdateRateHz() override {return 30.f;}

private:
  rclcpp::Publisher<px4_msgs::msg::PathSetpoint>::SharedPtr _pub;
};

/**
 * @brief Setpoint type for fixed-wing path following in the global (lat/lon/alt) frame.
 *
 * Accepts a path position in global WGS-84 coordinates, converts it to local
 * NED via the vehicle's map projection, then delegates to FwPathSetpointType.
 * This is the global-frame analogue of FwPathSetpointType, following the same
 * pattern as MulticopterGotoGlobalSetpointType vs MulticopterGotoSetpointType.
 */
class FwGlobalPathSetpointType
{
public:
  explicit FwGlobalPathSetpointType(Context & context);
  ~FwGlobalPathSetpointType() = default;

  /**
   * @brief Update the global path setpoint.
   *
   * @param global_position      latitude [deg], longitude [deg], altitude AMSL [m]
   * @param tangent_ned          Path tangent unit vector in NED frame; unset = not controlled
   * @param height_rate          Desired height rate [m/s]; unset = not controlled
   * @param curvature            Path curvature [1/m]; unset = not controlled
   * @param equivalent_airspeed  Target equivalent airspeed [m/s]; unset = not controlled
   */
  void update(
    const Eigen::Vector3d & global_position,
    const std::optional<Eigen::Vector3f> & tangent_ned = {},
    const std::optional<float> & height_rate = {},
    const std::optional<float> & curvature = {},
    const std::optional<float> & equivalent_airspeed = {});

private:
  rclcpp::Node & _node;
  std::unique_ptr<MapProjection> _map_projection;
  std::shared_ptr<FwPathSetpointType> _path_setpoint;
};

/** @}*/
} /* namespace px4_ros2 */
