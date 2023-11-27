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

using Eigen::Vector2d;
using px4_msgs::msg::VehicleGlobalPosition;

struct GlobalPositionEstimate
{
  rclcpp::Time timestamp_sample {};

  // Lat lon
  std::optional<Vector2d> lat_lon {std::nullopt};
  // Variance of horizontal position error (metres)
  std::optional<float> horizontal_variance {std::nullopt};

  // Altitude (MSL frame)
  std::optional<float> altitude_msl {std::nullopt};
  // Variance of vertical position error (meters)
  std::optional<float> vertical_variance {std::nullopt};
};

class GlobalNavigationInterface : public NavigationInterfaceBase<GlobalPositionEstimate>
{
public:
  explicit GlobalNavigationInterface(rclcpp::Node & node);
  ~GlobalNavigationInterface() override = default;

  /**
   * @brief Publish global position estimate to FMU.
   */
  NavigationInterfaceReturnCode update(const GlobalPositionEstimate & global_position_estimate)
  const override;

private:
  /**
   * @brief Check that at least one estimate value is defined.
   */
  bool isEstimateNonEmpty(const GlobalPositionEstimate & estimate) const override;

  /**
   * @brief Check that if an estimate value is defined, its variance is also defined and strictly greater than zero.
   */
  bool isVarianceValid(const GlobalPositionEstimate & estimate) const override;

  /**
   * @brief Check that if an estimate value is defined, its associated frame is not *FRAME_UNKNOWN.
   */
  bool isFrameValid(const GlobalPositionEstimate & estimate) const override;

  /**
   * @brief Check that if an estimate value is defined, none of its fields are NAN.
   */
  bool isValueNotNAN(const GlobalPositionEstimate & estimate) const override;

  rclcpp::Publisher<VehicleGlobalPosition>::SharedPtr _aux_global_position_pub;

  // uint8_t _altitude_frame;
};

} // namespace px4_ros2
