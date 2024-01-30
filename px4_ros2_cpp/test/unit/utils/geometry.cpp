/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "util.hpp"

#include <gtest/gtest.h>
#include <px4_ros2/utils/geometry.hpp>


TEST(Geometry, wrapPi) {
  EXPECT_NEAR(M_PI_2, px4_ros2::wrapPi(M_PI_2), 1e-3);
  EXPECT_NEAR(-M_PI_2, px4_ros2::wrapPi(-M_PI_2), 1e-3);
  EXPECT_NEAR(0.F, px4_ros2::wrapPi(4 * M_PI), 1e-3);
  EXPECT_NEAR(-3.F * M_PI / 4.F, px4_ros2::wrapPi(-11.F * M_PI / 4.F), 1e-3);
  EXPECT_NEAR(3.F * M_PI / 4.F, px4_ros2::wrapPi(11.F * M_PI / 4.F), 1e-3);

  // Absolute value due to floating point precision, 2*k*pi can map to -pi and pi
  EXPECT_NEAR(M_PI, std::fabs(px4_ros2::wrapPi(-11.0 * M_PI)), 1e-3);
}

TEST(Geometry, quaternionToEulerRPY) {
  Eigen::Quaternionf q;
  Eigen::Vector3f v_euler;

  // Roll
  q = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX());
  v_euler = Eigen::Vector3f{M_PI_2, 0.F, 0.F};
  vectorsApproxEqualTest(v_euler, px4_ros2::quaternionToEulerRPY(q), "roll pi/2");

  // Pitch
  q = Eigen::AngleAxisf(-M_PI / 4.F, Eigen::Vector3f::UnitY());
  v_euler = Eigen::Vector3f{0.F, -M_PI / 4.F, 0.F};
  vectorsApproxEqualTest(v_euler, px4_ros2::quaternionToEulerRPY(q), "pitch -pi/4");

  // Yaw
  q = Eigen::AngleAxisf(3.F * M_PI / 4.F, Eigen::Vector3f::UnitZ());
  v_euler = Eigen::Vector3f{0.F, 0.F, 3.F * M_PI / 4.F};
  vectorsApproxEqualTest(v_euler, px4_ros2::quaternionToEulerRPY(q), "yaw 3pi/4");

  // Gimbal lock cases
  q = Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitY());
  v_euler = Eigen::Vector3f{0.F, -M_PI_2, 0.F};
  vectorsApproxEqualTest(v_euler, px4_ros2::quaternionToEulerRPY(q), "gimbal lock: pitch -pi/2");

  q = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitY());
  v_euler = Eigen::Vector3f{0.F, M_PI_2, 0.F};
  vectorsApproxEqualTest(v_euler, px4_ros2::quaternionToEulerRPY(q), "gimbal lock: pitch pi/2");

  // Multi-axis rotations have multiple euler angle representations
  // Therefore we convert euler angle back to quaternion and compare to that

  q = Eigen::AngleAxisf(0.1F, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1F, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1F, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);

  q = Eigen::AngleAxisf(0.1F, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1F, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(M_PI - 0.1F, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);

  q = Eigen::AngleAxisf(0.1F, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1F, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(M_PI + 0.1F, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);

  q = Eigen::AngleAxisf(-0.1F, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1F, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.1F, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);

  q = Eigen::AngleAxisf(-0.1F, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1F, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1F, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);

  q = Eigen::AngleAxisf(-0.1F, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1F, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-M_PI + 0.1F, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);

  q = Eigen::AngleAxisf(0.1F, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1F, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-M_PI - 0.1F, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);

  q = Eigen::AngleAxisf(0.1F, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1F, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.1F, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);
}

TEST(Geometry, quaternionToRoll) {
  Eigen::Quaternionf q;
  float roll;

  // 90 deg roll
  q = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX());
  roll = M_PI_2;
  EXPECT_NEAR(roll, px4_ros2::quaternionToRoll(q), 1e-3);

  // Rolled -45 deg along Y-axis
  q = Eigen::AngleAxisf(-M_PI / 4.F, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());
  roll = -M_PI / 4.F;
  EXPECT_NEAR(roll, px4_ros2::quaternionToRoll(q), 1e-3);
}

TEST(Geometry, quaternionToPitch) {
  Eigen::Quaternionf q;
  float pitch;

  // 90 deg pitch
  q = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitY());
  pitch = M_PI_2;
  EXPECT_NEAR(pitch, px4_ros2::quaternionToPitch(q), 1e-3);

  // Rolled -45 deg along Y-axis
  q = Eigen::AngleAxisf(-M_PI / 4.F, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());
  pitch = 0.F;
  EXPECT_NEAR(pitch, px4_ros2::quaternionToPitch(q), 1e-3);
}

TEST(Geometry, quaternionToYaw) {
  Eigen::Quaternionf q;
  float yaw;

  // 90 deg yaw
  q = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());
  yaw = M_PI_2;
  EXPECT_NEAR(yaw, px4_ros2::quaternionToYaw(q), 1e-3);

  // Rolled -45 deg along Y-axis
  q = Eigen::AngleAxisf(-M_PI / 4.F, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());
  yaw = M_PI_2;
  EXPECT_NEAR(yaw, px4_ros2::quaternionToYaw(q), 1e-3);
}
