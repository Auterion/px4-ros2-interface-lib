/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/global_path_setpoint.hpp>
#include <Eigen/Eigen>
#include <optional>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types
 *  @{
 */

struct FwGlobalPathSetpoint;

/**
 * @brief Setpoint type for fixed-wing path following in the global (lat/lon/alt) frame.
 *
 * Publishes a GlobalPathSetpoint to the PX4 guidance controller. The setpoint
 * describes a path in WGS-84 coordinates with an optional tangent direction,
 * height rate, and curvature. Setting any field to NaN (by leaving it unset)
 * tells PX4 not to control that quantity.
 */
class FwGlobalPathSetpointType : public SetpointBase
{
public:
  explicit FwGlobalPathSetpointType(Context & context);

  ~FwGlobalPathSetpointType() override = default;

  Configuration getConfiguration() override;

  /**
   * @brief Publish a global path setpoint.
   *
   * @param setpoint A FwGlobalPathSetpoint describing the desired path point.
   *   Fields left as std::nullopt are sent as NaN, which tells PX4 not to
   *   control that quantity. See
   *   https://docs.px4.io/main/en/msg_docs/GlobalPathSetpoint.html for details.
   */
  void update(const FwGlobalPathSetpoint & setpoint);

  float desiredUpdateRateHz() override {return 30.f;}

private:
  rclcpp::Publisher<px4_msgs::msg::GlobalPathSetpoint>::SharedPtr _global_path_sp_pub;
};

/**
 * @brief Holds the fields of a GlobalPathSetpoint.
 *
 * All fields are optional; unset fields are published as NaN, which tells the
 * PX4 guidance controller not to control that quantity.
 */
struct FwGlobalPathSetpoint
{
  std::optional<double> lat;
  std::optional<double> lon;
  std::optional<float> alt;
  std::optional<Eigen::Vector3f> tangent;
  std::optional<float> height_rate;
  std::optional<float> curvature;

  FwGlobalPathSetpoint & withLatLon(double lat_deg, double lon_deg)
  {
    lat = lat_deg;
    lon = lon_deg;
    return *this;
  }

  FwGlobalPathSetpoint & withAltitude(float alt_amsl)
  {
    alt = alt_amsl;
    return *this;
  }

  FwGlobalPathSetpoint & withTangent(const Eigen::Vector3f & tangent_ned)
  {
    tangent = tangent_ned;
    return *this;
  }

  FwGlobalPathSetpoint & withHeightRate(float height_rate_sp)
  {
    height_rate = height_rate_sp;
    return *this;
  }

  FwGlobalPathSetpoint & withCurvature(float curvature_sp)
  {
    curvature = curvature_sp;
    return *this;
  }
};

/** @}*/
} /* namespace px4_ros2 */
