/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_ros2/navigation/experimental/navigation_interface_base.hpp>

namespace px4_ros2
{

/**
 * @struct GlobalPositionEstimate
 * @brief Represents a global position estimate to be passed to `GlobalNavigationInterface::update`.
 *
 * This struct holds information about the global position estimate, including: the timestamp of the sample, latitude and longitude, altitude in the MSL frame, and their associated variances.
 * @see GlobalNavigationInterface::update
 */
struct GlobalPositionEstimate
{
  /** @brief Timestamp of the sample. */
  rclcpp::Time timestamp_sample {};

  /** @brief Latitude and longitude. */
  std::optional<Eigen::Vector2d> lat_lon {std::nullopt};
  /** @brief Variance of horizontal position error (meters). */
  std::optional<float> horizontal_variance {std::nullopt};

  /** @brief Altitude in the MSL frame. */
  std::optional<float> altitude_msl {std::nullopt};
  /** @brief Variance of vertical position error (meters). */
  std::optional<float> vertical_variance {std::nullopt};
};

class GlobalNavigationInterface : public NavigationInterfaceBase
{
public:
  explicit GlobalNavigationInterface(rclcpp::Node & node);
  ~GlobalNavigationInterface() override = default;

  /**
   * @brief Publishes a global position estimate to the FMU.
   * The following are checked about the given global position estimate:
   * 1. The sample timestamp is defined.
   * 2. Values do not have a NAN.
   * 3. If an estimate value is provided, its associated variance value is well defined.
   * @param global_position_estimate The global position estimate to publish.
   */
  void update(const GlobalPositionEstimate & global_position_estimate) const;

private:
  /**
   * @brief Check that at least one estimate value is defined.
   */
  bool isEstimateNonEmpty(const GlobalPositionEstimate & estimate) const;

  /**
   * @brief Check that if an estimate value is defined, its variance is also defined and strictly greater than zero.
   */
  bool isVarianceValid(const GlobalPositionEstimate & estimate) const;

  /**
   * @brief Check that if an estimate value is defined, its associated frame is not *FRAME_UNKNOWN.
   */
  bool isFrameValid(const GlobalPositionEstimate & estimate) const;

  /**
   * @brief Check that if an estimate value is defined, none of its fields are NAN.
   */
  bool isValueNotNAN(const GlobalPositionEstimate & estimate) const;

  rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr _aux_global_position_pub;

  // uint8_t _altitude_frame;
};

} // namespace px4_ros2
