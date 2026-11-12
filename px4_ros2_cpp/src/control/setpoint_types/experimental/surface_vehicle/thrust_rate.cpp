/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/experimental/surface_vehicle/thrust_rate.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2 {

SurfaceVehicleThrustRateSetpointType::SurfaceVehicleThrustRateSetpointType(Context& context)
    : SetpointBase(context), _node(context.node())
{
  _surface_vehicle_thrust_setpoint_pub =
      context.node().create_publisher<px4_msgs::msg::SurfaceVehicleThrustSetpoint>(
          context.topicNamespacePrefix() + "fmu/in/surface_vehicle_thrust_setpoint" +
              px4_ros2::getMessageNameVersion<px4_msgs::msg::SurfaceVehicleThrustSetpoint>(),
          1);
  _surface_vehicle_rate_setpoint_pub =
      context.node().create_publisher<px4_msgs::msg::SurfaceVehicleRateSetpoint>(
          context.topicNamespacePrefix() + "fmu/in/surface_vehicle_rate_setpoint" +
              px4_ros2::getMessageNameVersion<px4_msgs::msg::SurfaceVehicleRateSetpoint>(),
          1);
}

void SurfaceVehicleThrustRateSetpointType::update(const float thrust_body_x,
                                                  const float yaw_rate_setpoint,
                                                  std::optional<float> thrust_body_y)
{
  onUpdate();

  px4_msgs::msg::SurfaceVehicleThrustSetpoint sp_thrust{};
  sp_thrust.x = thrust_body_x;
  sp_thrust.y = thrust_body_y.value_or(NAN);
  sp_thrust.timestamp = 0;  // Let PX4 set the timestamp
  _surface_vehicle_thrust_setpoint_pub->publish(sp_thrust);

  px4_msgs::msg::SurfaceVehicleRateSetpoint sp_rate{};
  sp_rate.yaw_rate_setpoint = yaw_rate_setpoint;
  sp_rate.timestamp = 0;  // Let PX4 set the timestamp
  _surface_vehicle_rate_setpoint_pub->publish(sp_rate);
}

}  // namespace px4_ros2
