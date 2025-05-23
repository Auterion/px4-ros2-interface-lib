/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/fixed_wing_lateral_setpoint.hpp>
#include <px4_msgs/msg/fixed_wing_longitudinal_setpoint.hpp>
#include <px4_msgs/msg/lateral_control_configuration.hpp>
#include <px4_msgs/msg/longitudinal_control_configuration.hpp>
#include <Eigen/Eigen>
#include <optional>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types
 *  @{
 */

struct FwLateralLongitudinalSetpoint;
struct FwControlConfiguration;

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
   * @brief Update the setpoint with full flexibility by passing a FwLateralLongitudinalSetpointType
   * and FwControlConfiguration struct.
   *
   * @param setpoint a FwLateralLongitudinalSetpoint object where course, airspeed
   * direction, lateral acceleration, altitude, height rate and equivalent airspeed
   * can be set for full flexibility. See https://docs.px4.io/main/en/msg_docs/FixedWingLateralSetpoint.html
   * and https://docs.px4.io/main/en/msg_docs/FixedWingLongitudinalSetpoint.html for more
   * information on this setpoint type.
   *
   * @param config a FwControlConfiguration object where pitch, throttle, lateral acceleration
   * configuration can be set. See https://docs.px4.io/main/en/msg_docs/LateralControlConfiguration.html
   * and https://docs.px4.io/main/en/msg_docs/LongitudinalControlConfiguration.html for more information
   * on how to configure the setpoint.
   *
   */

  void update(
    const FwLateralLongitudinalSetpoint & setpoint,
    const FwControlConfiguration & config);

  /**
 * @brief Update the setpoint with full flexibility by passing a FwLateralLongitudinalSetpointType
 *
 * @param setpoint a FwLateralLongitudinalSetpoint object where course, airspeed
 * direction, lateral acceleration, altitude, height rate and equivalent airspeed
 * can be set for full flexibility. See https://docs.px4.io/main/en/msg_docs/FixedWingLateralSetpoint.html
 * and https://docs.px4.io/main/en/msg_docs/FixedWingLongitudinalSetpoint.html for more
 * information on this setpoint type.
 *
 * @warning Any previously set configurations will be maintained when this method is called.
 * If no configurations have been previously set, PX4 will set default configuration.
 */

  void update(const FwLateralLongitudinalSetpoint & setpoint);

  /**
* @brief Update the setpoint with simplest set of inputs: Altitude and Course. Lateral acceleration setpoints will be used as feedforward when course is provided.
*  To directly control lateral acceleration, set course_sp to NAN.
*
* @warning Any previously set configurations will be maintained when this method is called.
* If no configurations have been previously set, PX4 will set default configurations.
*/
  void updateWithAltitude(
    float altitude_amsl_sp, float course_sp,
    std::optional<float> equivalent_airspeed_sp = {},
    std::optional<float> lateral_acceleration_sp = {});

  /**
* @brief Update the setpoint with simplest set of inputs: Height rate and Course. Lateral acceleration setpoints will be used as feedforward when course is provided.
*  To directly control lateral acceleration, set course_sp to NAN.
*
* @warning Any previously set configurations will be maintained when this method is called.
* If no configurations have been previously set, PX4 will set default configurations.
*/
  void updateWithHeightRate(
    float height_rate_sp, float course_sp,
    std::optional<float> equivalent_airspeed_sp = {},
    std::optional<float> lateral_acceleration_sp = {});

  float desiredUpdateRateHz() override {return 30.f;}

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::FixedWingLateralSetpoint>::SharedPtr
    _fw_lateral_sp_pub;
  rclcpp::Publisher<px4_msgs::msg::FixedWingLongitudinalSetpoint>::SharedPtr
    _fw_longitudinal_sp_pub;

  rclcpp::Publisher<px4_msgs::msg::LateralControlConfiguration>::SharedPtr
    _lateral_control_configuration_pub;
  rclcpp::Publisher<px4_msgs::msg::LongitudinalControlConfiguration>::SharedPtr
    _longitudinal_control_configuration_pub;
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

struct FwControlConfiguration
{
  std::optional<float> min_pitch;
  std::optional<float> max_pitch;
  std::optional<float> min_throttle;
  std::optional<float> max_throttle;
  std::optional<float> max_lateral_acceleration;
  std::optional<float> target_climb_rate;
  std::optional<float> target_sink_rate;

  FwControlConfiguration & withPitchLimits(float min_pitch_sp, float max_pitch_sp)
  {

    min_pitch = min_pitch_sp;
    max_pitch = max_pitch_sp;

    return *this;
  }

  FwControlConfiguration & withThrottleLimits(float min_throttle_sp, float max_throttle_sp)
  {

    min_throttle = min_throttle_sp;
    max_throttle = max_throttle_sp;

    return *this;
  }

  FwControlConfiguration & withMaxAcceleration(float max_lateral_acceleration_sp)
  {

    max_lateral_acceleration = max_lateral_acceleration_sp;
    return *this;
  }

  FwControlConfiguration & withTargetSinkRate(float target_sink_rate_sp)
  {

    target_sink_rate = target_sink_rate_sp;
    return *this;
  }

  FwControlConfiguration & withTargetClimbRate(float target_climb_rate_sp)
  {

    target_climb_rate = target_climb_rate_sp;
    return *this;
  }
};

/** @}*/
} /* namespace px4_ros2 */
