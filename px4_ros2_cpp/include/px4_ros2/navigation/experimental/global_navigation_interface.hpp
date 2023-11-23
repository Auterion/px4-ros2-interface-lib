/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>

#include <px4_msgs/msg/aux_global_position.hpp>
#include <px4_ros2/navigation/experimental/navigation_interface_base.hpp>
#include <px4_ros2/navigation/experimental/navigation_interface_common.hpp>

using namespace Eigen;
using namespace px4_msgs::msg;

namespace px4_ros2
{

struct GlobalPositionEstimate
{
  rclcpp::Time timestamp_sample {};

  // Lat lon
  std::optional<Vector2f> lat_lon {std::nullopt};

  // Altitude (AGL frame)
  std::optional<float> altitude_agl {std::nullopt};

  // Lat lon alt standard deviation
  std::optional<float> positional_uncertainty {std::nullopt};
};

class GlobalNavigationInterface : public NavigationInterfaceBase<GlobalPositionEstimate>
{
public:
  explicit GlobalNavigationInterface(Context & context);
  ~GlobalNavigationInterface() override = default;

  /**
   * @brief Publish global position estimate to FMU.
   */
  NavigationInterfaceReturnCode update(const GlobalPositionEstimate & global_position_estimate)
  const override;

  const std::string AUX_GLOBAL_POSITION_TOPIC = "/fmu/in/aux_global_position";

private:
  /**
   * @brief Check that at least one estimate value is defined.
   */
  bool _checkEstimateEmpty(const GlobalPositionEstimate & estimate) const override;

  /**
   * @brief Check that if an estimate value is defined, its variance is also defined and strictly greater than zero.
   */
  bool _checkVarianceValid(const GlobalPositionEstimate & estimate) const override;

  /**
   * @brief Check that if an estimate value is defined, its associated frame is not *FRAME_UNKNOWN.
   */
  bool _checkFrameValid(const GlobalPositionEstimate & estimate) const override;

  /**
   * @brief Check that if an estimate value is defined, none of its fields are NAN.
   */
  bool _checkValuesNotNAN(const GlobalPositionEstimate & estimate) const override;

  rclcpp::Node & _node;
  rclcpp::Publisher<AuxGlobalPosition>::SharedPtr _aux_global_position_pub;

  // uint8_t _altitude_frame;
};

} // namespace px4_ros2
