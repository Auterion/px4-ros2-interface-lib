/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "geometry.hpp"

#include <pybind11/eigen.h>

#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2_py/utils/conversion.hpp>

namespace py = pybind11;

void bindGeometry(py::module& m)
{
  m.def("rad_to_deg", &px4_ros2::radToDeg<double>);
  m.def("deg_to_rad", &px4_ros2::degToRad<double>);
  m.def("wrap_pi", &px4_ros2::wrapPi<double>);
  m.def(
      "quaternion_to_euler_rpy",
      [](const py::tuple& quat_tuple) {
        // There is no automatic type conversion for Eigen::Quaternion, so we use a tuple
        auto euler = px4_ros2::quaternionToEulerRpy<double>(quaternionFromTuple(quat_tuple));
        // Return as a tuple of (roll, pitch, yaw)
        return py::make_tuple(euler(0), euler(1), euler(2));
      },
      py::arg("quaternion").noconvert(),
      "Convert quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw) as a tuple");
  m.def(
      "euler_rpy_to_quaternion",
      [](double roll, double pitch, double yaw) {
        auto q = px4_ros2::eulerRpyToQuaternion<double>(roll, pitch, yaw);
        // Return as a tuple of (w, x, y, z)
        return py::make_tuple(q.w(), q.x(), q.y(), q.z());
      },
      py::arg("roll"), py::arg("pitch"), py::arg("yaw"),
      "Convert Euler angles (roll, pitch, yaw) to quaternion (w, x, y, z)");
  m.def(
      "quaternion_to_roll",
      [](const py::tuple& quat_tuple) {
        return px4_ros2::quaternionToRoll<double>(quaternionFromTuple(quat_tuple));
      },
      py::arg("quaternion").noconvert(), "Extract roll angle from quaternion (w, x, y, z)");

  m.def(
      "quaternion_to_pitch",
      [](const py::tuple& quat_tuple) {
        return px4_ros2::quaternionToPitch<double>(quaternionFromTuple(quat_tuple));
      },
      py::arg("quaternion").noconvert(), "Extract pitch angle from quaternion (w, x, y, z)");

  m.def(
      "quaternion_to_yaw",
      [](const py::tuple& quat_tuple) {
        return px4_ros2::quaternionToYaw<double>(quaternionFromTuple(quat_tuple));
      },
      py::arg("quaternion").noconvert(), "Extract yaw angle from quaternion (w, x, y, z)");
}
