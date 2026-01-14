/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "goto.hpp"

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <rclcpp/rclcpp.hpp>

namespace py = pybind11;

void bindSetpointMulticopterGoto(py::module& m)
{
  py::class_<px4_ros2::MulticopterGotoSetpointType,
             std::shared_ptr<px4_ros2::MulticopterGotoSetpointType>>
      goto_setpoint(m, "MulticopterGotoSetpointType");
  goto_setpoint.def(
      "update", &px4_ros2::MulticopterGotoSetpointType::update, py::arg("position"),
      py::arg("heading") = std::nullopt, py::arg("max_horizontal_speed") = std::nullopt,
      py::arg("max_vertical_speed") = std::nullopt, py::arg("max_heading_speed") = std::nullopt);

  // Constructor
  goto_setpoint.def(py::init<px4_ros2::Context&>(), py::arg("context"));
}
