/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <px4_msgs/msg/surface_vehicle_position_setpoint.hpp>
#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2 {
/** \ingroup setpoint_types_surface_vehicle
 *  @{
 */

/**
 * @brief Setpoint type for surface vehicle position control
 *
 * Can be used for surface vehicles, i.e. rovers and boats.
 */
class SurfaceVehiclePositionSetpointType : public SetpointBase {
 public:
  explicit SurfaceVehiclePositionSetpointType(Context& context);

  ~SurfaceVehiclePositionSetpointType() override = default;

  SetpointType getSetpointType() override
  {
    return px4_msgs::msg::SetpointConfig::TYPE_SURFACE_VEHICLE_POSITION;
  }

  float desiredUpdateRateHz() override { return 30.f; }

  /**
   * @brief Send a position setpoint to the flight controller.
   *
   * @param target_ned [m] Target position in NED frame. Takes values in [-inf, inf].
   * @param start_ned [m] Start position which specifies a line for the vehicle to track (Optional,
   * defaults to vehicle position). Takes values in [-inf, inf].
   * @param cruising_speed [m/s] Cruising speed (Optional, defaults to maximum speed). Takes values
   * in [0, inf].
   * @param arrival_speed [m/s] Speed the vehicle should arrive at the target with (Optional,
   * defaults to zero). Takes values in [0, inf].
   * @param yaw [rad] Omnidirectional vehicles only: Specify vehicle yaw during travel (Optional,
   * defaults to vehicle yaw). Takes values in [-pi,pi].
   */
  void update(const Eigen::Vector2f& target_ned,
              const std::optional<Eigen::Vector2f>& start_ned = {},
              std::optional<float> cruising_speed = {}, std::optional<float> arrival_speed = {},
              std::optional<float> yaw = {});

 private:
  rclcpp::Node& _node;
  rclcpp::Publisher<px4_msgs::msg::SurfaceVehiclePositionSetpoint>::SharedPtr
      _surface_vehicle_position_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
