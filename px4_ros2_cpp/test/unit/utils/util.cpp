/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "util.hpp"
#include <gtest/gtest.h>

void quaternionsApproxEqualTest(
  const Eigen::Quaternionf & q_expected,
  const Eigen::Quaternionf & q_actual, const std::string & msg, const float precision)
{
  // quaternion with all inverted components is equivalent to itself
  Eigen::Quaternionf q_expected_negated {-q_expected.w(), -q_expected.x(), -q_expected.y(),
    -q_expected.z()};

  EXPECT_TRUE(
    q_expected.isApprox(
      q_actual,
      precision) || q_expected_negated.isApprox(q_actual, precision))
    << "test: " << msg << std::endl
    << "  q_actual: " << q_actual.coeffs().transpose() << std::endl
    << "q_expected: " << q_expected.coeffs().transpose() << std::endl;
}

void vectorsApproxEqualTest(
  const Eigen::Vector3d & v_expected, const Eigen::Vector3d & v_actual,
  const std::string & msg, const double precision)
{
  EXPECT_TRUE(v_expected.isApprox(v_actual, precision))
    << "test: " << msg << std::endl
    << "  v_actual: " << v_actual.transpose() << std::endl
    << "v_expected: " << v_expected.transpose() << std::endl;
}

void vectorsApproxEqualTest(
  const Eigen::Vector3f & v_expected, const Eigen::Vector3f & v_actual,
  const std::string & msg, const float precision)
{
  vectorsApproxEqualTest(
    v_expected.cast<double>(),
    v_actual.cast<double>(),
    msg, static_cast<double>(precision));
}

void quaternionToEulerReconstructionTest(const Eigen::Quaternionf & q, const std::string & msg)
{
  Eigen::Vector3f v_euler = px4_ros2::quaternionToEulerRpy(q);
  Eigen::Quaternionf reconstructed_quaternion =
    Eigen::AngleAxisf(v_euler.z(), Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(v_euler.y(), Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(v_euler.x(), Eigen::Vector3f::UnitX());
  quaternionsApproxEqualTest(q, reconstructed_quaternion, msg, 1e-3);
}
