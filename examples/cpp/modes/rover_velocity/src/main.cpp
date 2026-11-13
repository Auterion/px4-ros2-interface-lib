/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "rclcpp/rclcpp.hpp"

// The surface vehicle setpoint types are only available if px4_msgs provides the
// SurfaceVehicle* messages (not the case for older px4_msgs versions)
#if __has_include(<px4_ros2/control/setpoint_types/experimental/surface_vehicle/speed_rate.hpp>)

#include <mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

using MyNodeWithMode = px4_ros2::NodeWithMode<RoverVelocityMode>;

static const std::string kNodeName = "rover_velocity_mode";
static const bool kEnableDebugOutput = true;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNodeWithMode>(kNodeName, kEnableDebugOutput));
  rclcpp::shutdown();
  return 0;
}

#else

int main(int argc, char* argv[])
{
  RCLCPP_ERROR(rclcpp::get_logger("rover_velocity_mode"),
               "px4_ros2_cpp was built without the surface vehicle setpoint types "
               "(px4_msgs does not provide the SurfaceVehicle* messages)");
  return 1;
}

#endif
