/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <src/components/registration.hpp>

class FakeRegistration : public Registration
{
public:
  explicit FakeRegistration(rclcpp::Node & node)
  : Registration(node)
  {}
  ~FakeRegistration() override = default;

  bool doRegister(const RegistrationSettings & settings) override
  {
    setRegistrationDetails(_arming_check_id, _mode_id, _mode_executor_id);
    return true;
  }
  void doUnregister() override {}

private:
  const int _arming_check_id{1};
  const px4_ros2::ModeBase::ModeID _mode_id{100};
  const int _mode_executor_id{1};
};
