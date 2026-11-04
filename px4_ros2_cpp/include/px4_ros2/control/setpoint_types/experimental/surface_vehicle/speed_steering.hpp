/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/surface_vehicle_speed_setpoint.hpp>
#include <px4_msgs/msg/surface_vehicle_steering_setpoint.hpp>
#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2 {
/** \ingroup setpoint_types_surface_vehicle
 *  @{
 */

/**
 * @brief Setpoint type for surface vehicle speed and steering control
 *
 * Can be used for surface vehicles, i.e. rovers and boats.
 */
class SurfaceVehicleSpeedSteeringSetpointType : public SetpointBase {
 public:
  explicit SurfaceVehicleSpeedSteeringSetpointType(Context& context);

  ~SurfaceVehicleSpeedSteeringSetpointType() override = default;

  SetpointType getSetpointType() override
  {
    return px4_msgs::msg::SetpointConfig::TYPE_SURFACE_VEHICLE_SPEED_STEERING;
  }

  float desiredUpdateRateHz() override { return 30.f; }

  /**
   * @brief Send a speed setpoint and a steering setpoint to the flight controller.
   *
   * @param speed_body_x [m/s] Speed setpoint in body x direction. Takes values in [-inf
   * (Backwards), inf (Forwards)].
   * @param normalized_steering_setpoint [-] Normalized steering setpoint. Ackermann rovers:
   * Normalized steering angle, Differential/Mecanum rovers: Normalized speed difference between
   * the left and right wheels, Boats: Normalized steering command. Takes values in [-1 (Left), 1
   * (Right)].
   * @param speed_body_y [m/s] Speed setpoint in body y direction (Optional, only relevant for
   * mecanum rovers, ignored by all other vehicles). Takes values in [-inf (Left), inf (Right)].
   */
  void update(float speed_body_x, float normalized_steering_setpoint,
              std::optional<float> speed_body_y = {});

 private:
  rclcpp::Node& _node;
  rclcpp::Publisher<px4_msgs::msg::SurfaceVehicleSpeedSetpoint>::SharedPtr
      _surface_vehicle_speed_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::SurfaceVehicleSteeringSetpoint>::SharedPtr
      _surface_vehicle_steering_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
