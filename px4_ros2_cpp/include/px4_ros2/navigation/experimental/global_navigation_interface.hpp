/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>

#include <px4_msgs/msg/aux_global_position.hpp>
#include <px4_ros2/navigation/experimental/navigation_interface_codes.hpp>

using namespace Eigen;
using namespace px4_msgs::msg;

namespace px4_ros2
{

struct GlobalPositionEstimate
{
  uint64_t timestamp_sample {};

};

class GlobalNavigationInterface
{
public:
  explicit GlobalNavigationInterface(rclcpp::Node & node, uint8_t altitude_frame);
  ~GlobalNavigationInterface() = default;

  /**
   * @brief Publish global position estimate to FMU.
   */
  int update(const GlobalPositionEstimate & global_position_estimate) const;

  const std::string AUX_GLOBAL_POSITION_TOPIC = "/fmu/in/aux_global_position";

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<AuxGlobalPosition>::SharedPtr _aux_global_position_pub;

  uint8_t _altitude_frame;
};

} // namespace px4_ros2
