/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/surface_vehicle_rate_setpoint.hpp>
#include <px4_msgs/msg/surface_vehicle_speed_setpoint.hpp>
#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2 {
/** \ingroup setpoint_types_surface_vehicle
 *  @{
 */

/**
 * @brief Setpoint type for surface vehicle speed and rate control
 *
 * Can be used for surface vehicles, i.e. rovers and boats.
 */
class SurfaceVehicleSpeedRateSetpointType : public SetpointBase {
 public:
  explicit SurfaceVehicleSpeedRateSetpointType(Context& context);

  ~SurfaceVehicleSpeedRateSetpointType() override = default;

  SetpointType getSetpointType() override
  {
    return px4_msgs::msg::SetpointConfig::TYPE_SURFACE_VEHICLE_SPEED_RATE;
  }

  float desiredUpdateRateHz() override { return 30.f; }

  /**
   * @brief Send a speed setpoint and a yaw rate setpoint to the flight controller.
   *
   * @param speed_body_x [m/s] Speed setpoint in body x direction. Takes values in [-inf
   * (Backwards), inf (Forwards)].
   * @param yaw_rate_setpoint [rad/s] Yaw rate setpoint in NED frame. Takes values in [-inf, inf].
   * @param speed_body_y [m/s] Speed setpoint in body y direction (Optional, only relevant for
   * omnidirectional vehicles, ignored by all other vehicles). Takes values in [-inf (Left), inf
   * (Right)].
   */
  void update(float speed_body_x, float yaw_rate_setpoint, std::optional<float> speed_body_y = {});

 private:
  rclcpp::Node& _node;
  rclcpp::Publisher<px4_msgs::msg::SurfaceVehicleSpeedSetpoint>::SharedPtr
      _surface_vehicle_speed_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::SurfaceVehicleRateSetpoint>::SharedPtr
      _surface_vehicle_rate_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
