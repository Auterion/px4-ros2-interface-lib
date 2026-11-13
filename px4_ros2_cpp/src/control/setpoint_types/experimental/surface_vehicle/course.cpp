/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/experimental/surface_vehicle/course.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2 {

SurfaceVehicleCourseSetpointType::SurfaceVehicleCourseSetpointType(Context& context)
    : SetpointBase(context), _node(context.node())
{
  _surface_vehicle_course_setpoint_pub =
      context.node().create_publisher<px4_msgs::msg::SurfaceVehicleCourseSetpoint>(
          context.topicNamespacePrefix() + "fmu/in/surface_vehicle_course_setpoint" +
              px4_ros2::getMessageNameVersion<px4_msgs::msg::SurfaceVehicleCourseSetpoint>(),
          1);
}

void SurfaceVehicleCourseSetpointType::update(const float course, std::optional<float> speed,
                                              std::optional<float> yaw)
{
  onUpdate();

  px4_msgs::msg::SurfaceVehicleCourseSetpoint sp{};
  sp.course = course;
  sp.speed = speed.value_or(NAN);
  sp.yaw = yaw.value_or(NAN);
  sp.timestamp = 0;  // Let PX4 set the timestamp
  _surface_vehicle_course_setpoint_pub->publish(sp);
}

}  // namespace px4_ros2
