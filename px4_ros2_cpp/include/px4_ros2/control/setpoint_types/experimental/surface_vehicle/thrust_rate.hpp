/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#if defined(PX4_ROS2_HAS_SURFACE_VEHICLE_MSGS) && !PX4_ROS2_HAS_SURFACE_VEHICLE_MSGS
#error "This setpoint type requires a px4_msgs version that provides the SurfaceVehicle* messages"
#endif

#include <px4_msgs/msg/surface_vehicle_rate_setpoint.hpp>
#include <px4_msgs/msg/surface_vehicle_thrust_setpoint.hpp>
#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2 {
/** \ingroup setpoint_types_surface_vehicle
 *  @{
 */

/**
 * @brief Setpoint type for surface vehicle thrust and rate control
 *
 * Can be used for surface vehicles, i.e. rovers and boats.
 */
class SurfaceVehicleThrustRateSetpointType : public SetpointBase {
 public:
  explicit SurfaceVehicleThrustRateSetpointType(Context& context);

  ~SurfaceVehicleThrustRateSetpointType() override = default;

  SetpointType getSetpointType() override
  {
    return px4_msgs::msg::SetpointConfig::TYPE_SURFACE_VEHICLE_THRUST_RATE;
  }

  float desiredUpdateRateHz() override { return 30.f; }

  /**
   * @brief Send a thrust setpoint and a rate setpoint to the flight controller.
   *
   * @param thrust_body_x [-] Normalized thrust setpoint along body X axis. Takes values in [-1
   * (Backwards), 1 (Forwards)].
   * @param yaw_rate_setpoint [rad/s] Yaw rate setpoint in NED frame. Takes values in [-inf, inf].
   * @param thrust_body_y [-] Normalized thrust setpoint along body Y axis (Optional, only relevant
   * for omnidirectional vehicles, ignored by all other vehicles). Takes values in [-1 (Left), 1
   * (Right)].
   */
  void update(float thrust_body_x, float yaw_rate_setpoint,
              std::optional<float> thrust_body_y = {});

 private:
  rclcpp::Node& _node;
  rclcpp::Publisher<px4_msgs::msg::SurfaceVehicleThrustSetpoint>::SharedPtr
      _surface_vehicle_thrust_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::SurfaceVehicleRateSetpoint>::SharedPtr
      _surface_vehicle_rate_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
