/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

namespace px4_ros2::util
{
// helper type for std::visit
template<class ... Ts>
struct Overloaded : Ts ...
{
  using Ts::operator() ...;
};
// explicit deduction guide (not needed as of C++20)
template<class ... Ts>
Overloaded(Ts ...)->Overloaded<Ts...>;
} // namespace px4_ros2::util
