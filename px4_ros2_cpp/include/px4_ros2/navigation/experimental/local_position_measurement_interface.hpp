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

namespace px4_ros2
{

using AuxLocalPosition = px4_msgs::msg::VehicleOdometry;

enum class PoseFrame
{
  Unknown,
  LocalNED,
  LocalFRD
};

enum class VelocityFrame
{
  Unknown,
  LocalNED,
  LocalFRD,
  BodyFRD
};

constexpr inline uint8_t poseFrameToMessageFrame(px4_ros2::PoseFrame frame) noexcept
{
  switch (frame) {
    case PoseFrame::Unknown: return AuxLocalPosition::POSE_FRAME_UNKNOWN;

    case PoseFrame::LocalNED: return AuxLocalPosition::POSE_FRAME_NED;

    case PoseFrame::LocalFRD: return AuxLocalPosition::POSE_FRAME_FRD;
  }

  return AuxLocalPosition::POSE_FRAME_UNKNOWN;
}

constexpr inline uint8_t velocityFrameToMessageFrame(px4_ros2::VelocityFrame frame) noexcept
{
  switch (frame) {
    case VelocityFrame::Unknown: return AuxLocalPosition::VELOCITY_FRAME_UNKNOWN;

    case VelocityFrame::LocalNED: return AuxLocalPosition::VELOCITY_FRAME_NED;

    case VelocityFrame::LocalFRD: return AuxLocalPosition::VELOCITY_FRAME_FRD;

    case VelocityFrame::BodyFRD: return AuxLocalPosition::VELOCITY_FRAME_BODY_FRD;
  }

  return AuxLocalPosition::POSE_FRAME_UNKNOWN;
}

/**
 * @struct LocalPositionMeasurement
 * @brief Represents a local position measurement to be passed to `LocalPositionMeasurementInterface::update`.
 *
 * This struct holds information about the local position measurement, including: the timestamp of the sample, vertical and horizontal postion and velocity, attitude, and their associated variances.
 * @see LocalPositionMeasurementInterface::update
 */
struct LocalPositionMeasurement
{
  /** @brief Timestamp of the sample. */
  rclcpp::Time timestamp_sample {};

  /** @brief Position in the xy-plane. */
  std::optional<Eigen::Vector2f> position_xy {std::nullopt};
  /** @brief Variance of position error in the xy-plane. */
  std::optional<Eigen::Vector2f> position_xy_variance {std::nullopt};
  /** @brief Position in the z-axis. */
  std::optional<float> position_z {std::nullopt};
  /** @brief Variance of position error in the z-axis. */
  std::optional<float> position_z_variance {std::nullopt};

  /** @brief Velocity in the xy-plane. */
  std::optional<Eigen::Vector2f> velocity_xy {std::nullopt};
  /** @brief Variance of velocity error in the xy-plane. */
  std::optional<Eigen::Vector2f> velocity_xy_variance {std::nullopt};
  /** @brief Velocity in the z-axis. */
  std::optional<float> velocity_z {std::nullopt};
  /** @brief Variance of velocity error in the z-axis. */
  std::optional<float> velocity_z_variance {std::nullopt};

  /** @brief Attitude quaternion [w, x, y, z], Hamiltonian convention. */
  std::optional<Eigen::Quaternionf> attitude_quaternion {std::nullopt};
  /** @brief Variance of attitude error in body frame. */
  std::optional<Eigen::Vector3f> attitude_variance {std::nullopt};
};

class LocalPositionMeasurementInterface : public PositionMeasurementInterfaceBase
{
public:
  explicit LocalPositionMeasurementInterface(
    rclcpp::Node & node, PoseFrame pose_frame,
    VelocityFrame velocity_frame);
  ~LocalPositionMeasurementInterface() override = default;

  /**
   * @brief Publishes a local position measurement to the FMU.
   * The following are checked about the given local position measurement:
   * 1. The sample timestamp is defined.
   * 2. Values do not have a NAN.
   * 3. If a measurement value is provided, its associated variance value is well defined.
   * 4. If a measurement value is provided, its associated reference frame is not unknown.
   * @param local_position_measurement The local position measurement to publish.
   */
  void update(const LocalPositionMeasurement & local_position_measurement) const;

private:
  /**
   * @brief Check that at least one measurement value is defined.
   */
  bool isMeasurementNonEmpty(const LocalPositionMeasurement & measurement) const;

  /**
   * @brief Check that if a measurement value is defined, its variance is also defined and strictly greater than zero.
   */
  bool isVarianceValid(const LocalPositionMeasurement & measurement) const;

  /**
   * @brief Check that if a measurement value is defined, its associated frame is not *FRAME_UNKNOWN.
   */
  bool isFrameValid(const LocalPositionMeasurement & measurement) const;

  /**
   * @brief Check that if a measurement value is defined, none of its fields are NAN.
   */
  bool isValueNotNAN(const LocalPositionMeasurement & measurement) const;

  rclcpp::Publisher<AuxLocalPosition>::SharedPtr _aux_local_position_pub;

  uint8_t _pose_frame;
  uint8_t _velocity_frame;
};

} // namespace px4_ros2
