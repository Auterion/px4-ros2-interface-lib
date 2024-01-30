/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <Eigen/Core>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types_experimental
 *  @{
 */

class RatesSetpointType : public SetpointBase
{
public:
  explicit RatesSetpointType(Context & context);

  ~RatesSetpointType() override = default;

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override {return 500.F;}

  void update(
    const Eigen::Vector3f & rate_setpoints_ned_rad,
    const Eigen::Vector3f & thrust_setpoint_frd);

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr _vehicle_rates_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */
