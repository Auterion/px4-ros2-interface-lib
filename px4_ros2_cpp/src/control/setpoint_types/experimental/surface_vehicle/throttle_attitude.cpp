/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/experimental/surface_vehicle/throttle_attitude.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2 {

SurfaceVehicleThrottleAttitudeSetpointType::SurfaceVehicleThrottleAttitudeSetpointType(
    Context& context)
    : SetpointBase(context), _node(context.node())
{
  _surface_vehicle_throttle_setpoint_pub =
      context.node().create_publisher<px4_msgs::msg::SurfaceVehicleThrottleSetpoint>(
          context.topicNamespacePrefix() + "fmu/in/surface_vehicle_throttle_setpoint" +
              px4_ros2::getMessageNameVersion<px4_msgs::msg::SurfaceVehicleThrottleSetpoint>(),
          1);
  _surface_vehicle_attitude_setpoint_pub =
      context.node().create_publisher<px4_msgs::msg::SurfaceVehicleAttitudeSetpoint>(
          context.topicNamespacePrefix() + "fmu/in/surface_vehicle_attitude_setpoint" +
              px4_ros2::getMessageNameVersion<px4_msgs::msg::SurfaceVehicleAttitudeSetpoint>(),
          1);
}

void SurfaceVehicleThrottleAttitudeSetpointType::update(const float throttle_body_x,
                                                        const float yaw_setpoint,
                                                        std::optional<float> throttle_body_y)
{
  onUpdate();

  px4_msgs::msg::SurfaceVehicleThrottleSetpoint sp_throttle{};
  sp_throttle.x = throttle_body_x;
  sp_throttle.y = throttle_body_y.value_or(NAN);
  sp_throttle.timestamp = 0;  // Let PX4 set the timestamp
  _surface_vehicle_throttle_setpoint_pub->publish(sp_throttle);

  px4_msgs::msg::SurfaceVehicleAttitudeSetpoint sp_att{};
  sp_att.yaw_setpoint = yaw_setpoint;
  sp_att.timestamp = 0;  // Let PX4 set the timestamp
  _surface_vehicle_attitude_setpoint_pub->publish(sp_att);
}

}  // namespace px4_ros2
