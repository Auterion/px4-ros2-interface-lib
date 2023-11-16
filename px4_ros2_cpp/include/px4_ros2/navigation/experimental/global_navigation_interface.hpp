/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <optional>
#include <px4_msgs/msg/aux_global_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>


using namespace Eigen;
using namespace px4_msgs::msg;

namespace px4_ros2
{

struct GlobalPositionEstimate
{
  uint64_t timestamp_sample {};

};

class GlobalNavigationInterface : public rclcpp::Node
{
public:
  explicit GlobalNavigationInterface(uint8_t altitude_frame);
  ~GlobalNavigationInterface();

  void update(GlobalPositionEstimate & global_position_estimate);

private:
  const std::string AUX_GLOBAL_POSITION_TOPIC = "/fmu/in/aux_global_position";

  rclcpp::Publisher<AuxGlobalPosition>::SharedPtr _aux_global_position_pub;

  uint8_t _altitude_frame;
};

} // namespace px4_ros2
