/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <optional>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>

using namespace Eigen;
using AuxLocalPosition = px4_msgs::msg::VehicleOdometry;

namespace px4_ros2
{

struct LocalPositionEstimate
{
  uint64_t timestamp_sample {};

  // Position
  std::optional<Vector2f> position_xy {std::nullopt};
  std::optional<Vector2f> position_xy_variance {std::nullopt};
  std::optional<float> position_z {std::nullopt};
  std::optional<float> position_z_variance {std::nullopt};

  // Velocity
  std::optional<Vector2f> velocity_xy {std::nullopt};
  std::optional<Vector2f> velocity_xy_variance {std::nullopt};
  std::optional<float> velocity_z {std::nullopt};
  std::optional<float> velocity_z_variance {std::nullopt};

  // Attitude
  std::optional<Quaternionf> attitude_quaternion {std::nullopt};
  std::optional<Vector3f> attitude_variance {std::nullopt};
};

class LocalNavigationInterface : public rclcpp::Node
{
public:
  explicit LocalNavigationInterface(uint8_t pose_frame, uint8_t velocity_frame);
  ~LocalNavigationInterface();

  void update(LocalPositionEstimate & local_position_estimate);

private:
  const std::string AUX_LOCAL_POSITION_TOPIC = "/fmu/in/vehicle_odometry";

  rclcpp::Publisher<AuxLocalPosition>::SharedPtr _aux_local_position_pub;

  uint8_t _pose_frame;
  uint8_t _velocity_frame;
};

} // namespace px4_ros2
