/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <pybind11/pybind11.h>

#include <Eigen/Eigen>

/**
 * @brief Get a quaternion from a pybind11 tuple
 *
 * There is no automatic conversion provided for this by pybind11.
 */
static inline Eigen::Quaterniond quaternionFromTuple(const pybind11::tuple& quat_tuple)
{
  if (quat_tuple.size() != 4) {
    throw std::runtime_error("Quaternion must be a tuple of 4 elements (w, x, y, z)");
  }
  const double w = quat_tuple[0].cast<double>();
  const double x = quat_tuple[1].cast<double>();
  const double y = quat_tuple[2].cast<double>();
  const double z = quat_tuple[3].cast<double>();
  return Eigen::Quaterniond(w, x, y, z);
}
