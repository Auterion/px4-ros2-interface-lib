/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cmath>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>

#include <rclcpp/rclcpp.hpp>

static const std::string kName = "FW Attitude Example";

class FwAttModeTest : public px4_ros2::ModeBase
{
public:
  explicit FwAttModeTest(rclcpp::Node & node)
  : ModeBase(node, kName)
  {
    _att_setpoint = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);

  }

  void onActivate() override {}

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    // Setting constant angles and thrust.
    _att_setpoint->update(-25.f * M_PI / 180.f, 2.2f * M_PI / 180.f, 0.f, {0.27f, 0.f, 0.f});

  }

private:
  std::shared_ptr<px4_ros2::AttitudeSetpointType> _att_setpoint;

};
