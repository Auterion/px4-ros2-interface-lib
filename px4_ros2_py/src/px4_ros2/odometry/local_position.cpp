/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "local_position.hpp"

#include <pybind11/eigen.h>
#include <pybind11/functional.h>

#include <px4_ros2/odometry/local_position.hpp>
#include <rclcpp/rclcpp.hpp>

namespace py = pybind11;

void bindLocalPosition(py::module& m)
{
  py::class_<px4_ros2::OdometryLocalPosition, std::shared_ptr<px4_ros2::OdometryLocalPosition>>
      local_position(m, "LocalPosition");

  local_position
      .def("on_update",
           [](px4_ros2::OdometryLocalPosition& self, const std::function<void()>& callback) {
             // We don't pass the message, otherwise we'd have to register the type as well
             self.onUpdate([callback](const auto& msg) { callback(); });
           })
      .def("position_ned", &px4_ros2::OdometryLocalPosition::positionNed)
      .def("position_xy_valid", &px4_ros2::OdometryLocalPosition::positionXYValid)
      .def("position_z_valid", &px4_ros2::OdometryLocalPosition::positionZValid);

  // Constructor
  local_position.def(py::init<px4_ros2::Context&, bool>(), py::arg("context"),
                     py::arg("local_position_is_optional") = false);
}
