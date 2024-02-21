/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <gtest/gtest.h>
#include <Eigen/Eigen>

#include <px4_ros2/utils/geometry.hpp>

void quaternionsApproxEqualTest(
  const Eigen::Quaternionf & q_expected,
  const Eigen::Quaternionf & q_actual, const std::string & msg = "", float precision = 1e-3);

void vectorsApproxEqualTest(
  const Eigen::Vector3f & v_expected, const Eigen::Vector3f & v_actual,
  const std::string & msg = "", float precision = 1e-3);

void vectorsApproxEqualTest(
  const Eigen::Vector3d & v_expected, const Eigen::Vector3d & v_actual,
  const std::string & msg = "", double precision = 1e-6);

void quaternionToEulerReconstructionTest(
  const Eigen::Quaternionf & q,
  const std::string & msg = "");
