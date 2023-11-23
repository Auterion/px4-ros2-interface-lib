/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros2/navigation/experimental/navigation_interface_base.hpp>
#include <px4_ros2/navigation/experimental/navigation_interface_common.hpp>

using namespace Eigen;
using AuxLocalPosition = px4_msgs::msg::VehicleOdometry;

namespace px4_ros2
{

struct LocalPositionEstimate
{
  rclcpp::Time timestamp_sample {};

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

class LocalNavigationInterface : public NavigationInterfaceBase<LocalPositionEstimate>
{
public:
  explicit LocalNavigationInterface(
    Context & context, const uint8_t pose_frame,
    const uint8_t velocity_frame);
  ~LocalNavigationInterface() override = default;

  /**
   * @brief Publish local position estimate to FMU.
   */
  NavigationInterfaceReturnCode update(const LocalPositionEstimate & local_position_estimate) const
  override;

  const std::string AUX_LOCAL_POSITION_TOPIC = "/fmu/in/vehicle_visual_odometry";

private:
  /**
   * @brief Check that at least one estimate value is defined.
   */
  bool _isEstimateNonEmpty(const LocalPositionEstimate & estimate) const override;

  /**
   * @brief Check that if an estimate value is defined, its variance is also defined and strictly greater than zero.
   */
  bool _isVarianceValid(const LocalPositionEstimate & estimate) const override;

  /**
   * @brief Check that if an estimate value is defined, its associated frame is not *FRAME_UNKNOWN.
   */
  bool _isFrameValid(const LocalPositionEstimate & estimate) const override;

  /**
   * @brief Check that if an estimate value is defined, none of its fields are NAN.
   */
  bool _isValueNotNAN(const LocalPositionEstimate & estimate) const override;

  rclcpp::Publisher<AuxLocalPosition>::SharedPtr _aux_local_position_pub;

  uint8_t _pose_frame;
  uint8_t _velocity_frame;

  static constexpr uint8_t _available_pose_frames[3] = {
    AuxLocalPosition::POSE_FRAME_UNKNOWN,
    AuxLocalPosition::POSE_FRAME_NED,
    AuxLocalPosition::POSE_FRAME_FRD
  };

  static constexpr uint8_t _available_velocity_frames[4] = {
    AuxLocalPosition::VELOCITY_FRAME_UNKNOWN,
    AuxLocalPosition::VELOCITY_FRAME_NED,
    AuxLocalPosition::VELOCITY_FRAME_FRD,
    AuxLocalPosition::VELOCITY_FRAME_BODY_FRD
  };

};

} // namespace px4_ros2
