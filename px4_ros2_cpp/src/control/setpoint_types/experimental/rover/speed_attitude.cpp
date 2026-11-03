/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/experimental/rover/speed_attitude.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2 {

RoverSpeedAttitudeSetpointType::RoverSpeedAttitudeSetpointType(Context& context)
    : SetpointBase(context), _node(context.node())
{
  _surface_vehicle_speed_setpoint_pub =
      context.node().create_publisher<px4_msgs::msg::SurfaceVehicleSpeedSetpoint>(
          context.topicNamespacePrefix() + "fmu/in/surface_vehicle_speed_setpoint" +
              px4_ros2::getMessageNameVersion<px4_msgs::msg::SurfaceVehicleSpeedSetpoint>(),
          1);
  _surface_vehicle_attitude_setpoint_pub =
      context.node().create_publisher<px4_msgs::msg::SurfaceVehicleAttitudeSetpoint>(
          context.topicNamespacePrefix() + "fmu/in/surface_vehicle_attitude_setpoint" +
              px4_ros2::getMessageNameVersion<px4_msgs::msg::SurfaceVehicleAttitudeSetpoint>(),
          1);
}

void RoverSpeedAttitudeSetpointType::update(const float speed_body_x, const float yaw_setpoint,
                                            std::optional<float> speed_body_y)
{
  onUpdate();

  px4_msgs::msg::SurfaceVehicleSpeedSetpoint sp_speed{};
  sp_speed.speed_body_x = speed_body_x;
  sp_speed.speed_body_y = speed_body_y.value_or(NAN);
  sp_speed.timestamp = 0;  // Let PX4 set the timestamp
  _surface_vehicle_speed_setpoint_pub->publish(sp_speed);

  px4_msgs::msg::SurfaceVehicleAttitudeSetpoint sp_att{};
  sp_att.yaw_setpoint = yaw_setpoint;
  sp_att.timestamp = 0;  // Let PX4 set the timestamp
  _surface_vehicle_attitude_setpoint_pub->publish(sp_att);
}

}  // namespace px4_ros2
