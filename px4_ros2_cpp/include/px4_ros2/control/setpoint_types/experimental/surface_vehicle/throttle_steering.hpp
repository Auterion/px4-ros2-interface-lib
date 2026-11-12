/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/surface_vehicle_steering_setpoint.hpp>
#include <px4_msgs/msg/surface_vehicle_throttle_setpoint.hpp>
#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2 {
/** \ingroup setpoint_types_surface_vehicle
 *  @{
 */

/**
 * @brief Setpoint type for surface vehicle throttle and steering control
 *
 * Can be used for surface vehicles, i.e. rovers and boats.
 */
class SurfaceVehicleThrottleSteeringSetpointType : public SetpointBase {
 public:
  explicit SurfaceVehicleThrottleSteeringSetpointType(Context& context);

  ~SurfaceVehicleThrottleSteeringSetpointType() override = default;

  SetpointType getSetpointType() override
  {
    return px4_msgs::msg::SetpointConfig::TYPE_SURFACE_VEHICLE_THROTTLE_STEERING;
  }

  float desiredUpdateRateHz() override { return 30.f; }

  /**
   * @brief Send a throttle setpoint and a steering setpoint to the flight controller.
   *
   * @param throttle_body_x [-] Throttle setpoint along body X axis. Takes values in [-1
   * (Backwards), 1 (Forwards)].
   * @param normalized_steering_setpoint [-] Ackermann: Normalized steering angle,
   * Differential/Mecanum: Normalized speed difference between the left and right wheels. Takes
   * values in [-1 (Left), 1 (Right)].
   * @param throttle_body_y [-] Throttle setpoint along body Y axis (Optional, only relevant for
   * omnidirectional vehicles, ignored by all other vehicles). Takes values in [-1 (Left), 1
   * (Right)].
   */
  void update(float throttle_body_x, float normalized_steering_setpoint,
              std::optional<float> throttle_body_y = {});

 private:
  rclcpp::Node& _node;
  rclcpp::Publisher<px4_msgs::msg::SurfaceVehicleThrottleSetpoint>::SharedPtr
      _surface_vehicle_throttle_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::SurfaceVehicleSteeringSetpoint>::SharedPtr
      _surface_vehicle_steering_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
