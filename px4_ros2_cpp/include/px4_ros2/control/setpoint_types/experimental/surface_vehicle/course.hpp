/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/surface_vehicle_course_setpoint.hpp>
#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2 {
/** \ingroup setpoint_types_surface_vehicle
 *  @{
 */

/**
 * @brief Setpoint type for surface vehicle course control
 *
 * Can be used for surface vehicles, i.e. rovers and boats.
 * The vehicle tracks a course over ground. There is no desired track, so a sideways offset is not
 * corrected. To follow a desired track, use SurfaceVehiclePositionSetpointType instead.
 */
class SurfaceVehicleCourseSetpointType : public SetpointBase {
 public:
  explicit SurfaceVehicleCourseSetpointType(Context& context);

  ~SurfaceVehicleCourseSetpointType() override = default;

  SetpointType getSetpointType() override
  {
    // The course setpoint is handled by the surface vehicle position controller
    return px4_msgs::msg::SetpointConfig::TYPE_SURFACE_VEHICLE_POSITION;
  }

  float desiredUpdateRateHz() override { return 30.f; }

  /**
   * @brief Send a course setpoint to the flight controller.
   *
   * @param course [rad] Course over ground to hold in NED frame. Takes values in [-pi, pi].
   * @param speed [m/s] Cruising speed (Optional, defaults to maximum speed). A negative value
   * drives the vehicle in reverse along the course. Takes values in [-inf, inf].
   * @param yaw [rad] Vehicles that can move sideways (e.g. mecanum rovers) only: vehicle yaw while
   * holding the course (Optional, currently unused by PX4). Takes values in [-pi, pi].
   */
  void update(float course, std::optional<float> speed = {}, std::optional<float> yaw = {});

 private:
  rclcpp::Node& _node;
  rclcpp::Publisher<px4_msgs::msg::SurfaceVehicleCourseSetpoint>::SharedPtr
      _surface_vehicle_course_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
