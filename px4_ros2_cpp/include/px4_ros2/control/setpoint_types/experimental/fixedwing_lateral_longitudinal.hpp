/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/fixed_wing_lateral_setpoint.hpp>
#include <px4_msgs/msg/fixed_wing_longitudinal_setpoint.hpp>
#include <px4_msgs/msg/lateral_control_limits.hpp>
#include <px4_msgs/msg/longitudinal_control_limits.hpp>
#include <Eigen/Eigen>
#include <optional>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types_experimental
 *  @{
 */

struct FwLateralLongitudinalSetpoint;
struct FwControlLimits;

/**
 * @brief Setpoint type for fixedwing control
*/
class FwLateralLongitudinalSetpointType : public SetpointBase
{
public:
  explicit FwLateralLongitudinalSetpointType(Context & context);

  ~FwLateralLongitudinalSetpointType() override = default;

  Configuration getConfiguration() override;

  /**
   * @brief Update
   *
   * Unset optional values are not controlled
   *
   */

  void update(const FwLateralLongitudinalSetpoint & setpoint, const FwControlLimits & limits);

  void update(const FwLateralLongitudinalSetpoint & setpoint);

  float desiredUpdateRateHz() override {return 30.f;}

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::FixedWingLateralSetpoint>::SharedPtr
    _fw_lateral_sp_pub;
  rclcpp::Publisher<px4_msgs::msg::FixedWingLongitudinalSetpoint>::SharedPtr
    _fw_longitudinal_sp_pub;

  rclcpp::Publisher<px4_msgs::msg::LateralControlLimits>::SharedPtr
    _lateral_control_limits_pub;
  rclcpp::Publisher<px4_msgs::msg::LongitudinalControlLimits>::SharedPtr
    _longitudinal_control_limits_pub;
};

struct FwLateralLongitudinalSetpoint
{
  std::optional<float> course;
  std::optional<float> airspeed_direction;
  std::optional<float> lateral_acceleration;
  std::optional<float> altitude_msl;
  std::optional<float> height_rate;
  std::optional<float> equivalent_airspeed;

  FwLateralLongitudinalSetpoint & withCourse(float course_sp)
  {
    course = course_sp;
    return *this;
  }

  FwLateralLongitudinalSetpoint & withAirspeedDirection(float airspeed_direction_sp)
  {
    airspeed_direction = airspeed_direction_sp;
    return *this;
  }

  FwLateralLongitudinalSetpoint & withLateralAcceleration(float lateral_acceleration_sp)
  {
    lateral_acceleration = lateral_acceleration_sp;
    return *this;
  }

  FwLateralLongitudinalSetpoint & withAltitude(float altitude_sp)
  {
    altitude_msl = altitude_sp;
    return *this;
  }

  FwLateralLongitudinalSetpoint & withHeightRate(float height_rate_sp)
  {
    height_rate = height_rate_sp;
    return *this;
  }

  FwLateralLongitudinalSetpoint & withEquivalentAirspeed(float equivalent_airspeed_sp)
  {
    equivalent_airspeed = equivalent_airspeed_sp;
    return *this;
  }
};

struct FwControlLimits
{
  std::optional<float> min_pitch;
  std::optional<float> max_pitch;
  std::optional<float> min_throttle;
  std::optional<float> max_throttle;
  std::optional<float> min_equivalent_airspeed;
  std::optional<float> max_equivalent_airspeed;
  std::optional<float> max_lateral_acceleration;
  std::optional<float> target_climb_rate;
  std::optional<float> target_sink_rate;

  FwControlLimits & withPitchLimits(float min_pitch_sp, float max_pitch_sp)
  {

    min_pitch = min_pitch_sp;
    max_pitch = max_pitch_sp;

    return *this;
  }

  FwControlLimits & withThrottleLimits(float min_throttle_sp, float max_throttle_sp)
  {

    min_throttle = min_throttle_sp;
    max_throttle = max_throttle_sp;

    return *this;
  }

  FwControlLimits & withMaxAcceleration(float max_lateral_acceleration_sp)
  {

    max_lateral_acceleration = max_lateral_acceleration_sp;
    return *this;
  }

  FwControlLimits & withTargetSinkRate(float target_sink_rate_sp)
  {

    target_sink_rate = target_sink_rate_sp;
    return *this;
  }

  FwControlLimits & withTargetClimbRate(float target_climb_rate_sp)
  {

    target_climb_rate = target_climb_rate_sp;
    return *this;
  }
};

/** @}*/
} /* namespace px4_ros2 */
