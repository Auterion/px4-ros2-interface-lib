/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <px4_msgs/msg/airspeed_validated.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/utils/subscription.hpp>

namespace px4_ros2
{
/** \ingroup odometry
 *  @{
 */

/**
 * @brief Provides access to the vehicle's airspeed estimates
 */
class OdometryAirspeed : public Subscription<px4_msgs::msg::AirspeedValidated>
{
public:
  explicit OdometryAirspeed(Context & context);

  float indicatedAirspeed() const
  {
    if (!lastValid()) {
      return NAN;
    }
    const px4_msgs::msg::AirspeedValidated & aspd = last();
    return aspd.indicated_airspeed_m_s;
  }

  float calibratedAirspeed() const
  {
    if (!lastValid()) {
      return NAN;
    }
    const px4_msgs::msg::AirspeedValidated & aspd = last();
    return aspd.calibrated_airspeed_m_s;
  }

  float trueAirspeed() const
  {
    if (!lastValid()) {
      return NAN;
    }
    const px4_msgs::msg::AirspeedValidated & aspd = last();
    return aspd.true_airspeed_m_s;
  }
};

/** @}*/
} /* namespace px4_ros2 */
