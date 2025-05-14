/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <exception>
#include <stdexcept>
#include <string>

namespace px4_ros2
{

class Exception : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

} // namespace px4_ros2
