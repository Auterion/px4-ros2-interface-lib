/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/experimental/rover/speed_steering.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2 {

RoverSpeedSteeringSetpointType::RoverSpeedSteeringSetpointType(Context& context)
    : SetpointBase(context), _node(context.node())
{
  _rover_speed_setpoint_pub =
      context.node().create_publisher<px4_msgs::msg::SurfaceVehicleSpeedSetpoint>(
          context.topicNamespacePrefix() + "fmu/in/surface_vehicle_speed_setpoint" +
              px4_ros2::getMessageNameVersion<px4_msgs::msg::SurfaceVehicleSpeedSetpoint>(),
          1);
  _rover_steering_setpoint_pub =
      context.node().create_publisher<px4_msgs::msg::SurfaceVehicleSteeringSetpoint>(
          context.topicNamespacePrefix() + "fmu/in/surface_vehicle_steering_setpoint" +
              px4_ros2::getMessageNameVersion<px4_msgs::msg::SurfaceVehicleSteeringSetpoint>(),
          1);
}

void RoverSpeedSteeringSetpointType::update(const float speed_body_x,
                                            const float normalized_steering_setpoint,
                                            std::optional<float> speed_body_y)
{
  onUpdate();

  px4_msgs::msg::SurfaceVehicleSpeedSetpoint sp_speed{};
  sp_speed.speed_body_x = speed_body_x;
  sp_speed.speed_body_y = speed_body_y.value_or(NAN);
  sp_speed.timestamp = 0;  // Let PX4 set the timestamp
  _rover_speed_setpoint_pub->publish(sp_speed);

  px4_msgs::msg::SurfaceVehicleSteeringSetpoint sp_steering{};
  sp_steering.normalized_steering_setpoint = normalized_steering_setpoint;
  sp_steering.timestamp = 0;  // Let PX4 set the timestamp
  _rover_steering_setpoint_pub->publish(sp_steering);
}

}  // namespace px4_ros2
