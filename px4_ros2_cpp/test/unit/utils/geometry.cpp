/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "util.hpp"

#include <gtest/gtest.h>
#include <px4_ros2/utils/geometry.hpp>

using namespace px4_ros2::literals; // NOLINT

TEST(Geometry, radToDeg) {
  EXPECT_NEAR(180.f, px4_ros2::radToDeg(M_PI), 1e-3);
  EXPECT_NEAR(-720.f, px4_ros2::radToDeg(-4.f * M_PI), 1e-3);
  EXPECT_NEAR(60.f, px4_ros2::radToDeg(M_PI / 3.f), 1e-3);

  // Test literals
  EXPECT_NEAR(90.f, px4_ros2::radToDeg(90.0_deg), 1e-3);
  EXPECT_NEAR(90.f, px4_ros2::radToDeg(1.57079_rad), 1e-3);
}

TEST(Geometry, degToRad) {
  EXPECT_NEAR(M_PI, px4_ros2::degToRad(180.f), 1e-3);
  EXPECT_NEAR(-4.f * M_PI, px4_ros2::degToRad(-720.f), 1e-3);
  EXPECT_NEAR(M_PI / 3.f, px4_ros2::degToRad(60.f), 1e-3);
}

TEST(Geometry, wrapPi) {
  EXPECT_NEAR(M_PI_2, px4_ros2::wrapPi(M_PI_2), 1e-3);
  EXPECT_NEAR(-M_PI_2, px4_ros2::wrapPi(-M_PI_2), 1e-3);
  EXPECT_NEAR(0.f, px4_ros2::wrapPi(4 * M_PI), 1e-3);
  EXPECT_NEAR(-3.f * M_PI / 4.f, px4_ros2::wrapPi(-11.f * M_PI / 4.f), 1e-3);
  EXPECT_NEAR(3.f * M_PI / 4.f, px4_ros2::wrapPi(11.f * M_PI / 4.f), 1e-3);

  // Absolute value due to floating point precision, 2*k*pi can map to -pi and pi
  EXPECT_NEAR(M_PI, std::fabs(px4_ros2::wrapPi(-11.0 * M_PI)), 1e-3);
}

TEST(Geometry, quaternionToEulerRpy) {
  Eigen::Quaternionf q;
  Eigen::Vector3f v_euler;

  // Roll
  q = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX());
  v_euler = Eigen::Vector3f{M_PI_2, 0.f, 0.f};
  vectorsApproxEqualTest(v_euler, px4_ros2::quaternionToEulerRpy(q), "roll pi/2");

  // Pitch
  q = Eigen::AngleAxisf(-M_PI / 4.f, Eigen::Vector3f::UnitY());
  v_euler = Eigen::Vector3f{0.f, -M_PI / 4.f, 0.f};
  vectorsApproxEqualTest(v_euler, px4_ros2::quaternionToEulerRpy(q), "pitch -pi/4");

  // Yaw
  q = Eigen::AngleAxisf(3.f * M_PI / 4.f, Eigen::Vector3f::UnitZ());
  v_euler = Eigen::Vector3f{0.f, 0.f, 3.f * M_PI / 4.f};
  vectorsApproxEqualTest(v_euler, px4_ros2::quaternionToEulerRpy(q), "yaw 3pi/4");

  // Gimbal lock cases
  q = Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitY());
  v_euler = Eigen::Vector3f{0.f, -M_PI_2, 0.f};
  vectorsApproxEqualTest(v_euler, px4_ros2::quaternionToEulerRpy(q), "gimbal lock: pitch -pi/2");

  q = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitY());
  v_euler = Eigen::Vector3f{0.f, M_PI_2, 0.f};
  vectorsApproxEqualTest(v_euler, px4_ros2::quaternionToEulerRpy(q), "gimbal lock: pitch pi/2");

  // Multi-axis rotations have multiple euler angle representations
  // Therefore we convert euler angle back to quaternion and compare to that

  q = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);

  q = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(M_PI - 0.1f, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);

  q = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(M_PI + 0.1f, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);

  q = Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);

  q = Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);

  q = Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-M_PI + 0.1f, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);

  q = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-M_PI - 0.1f, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);

  q = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitZ());
  quaternionToEulerReconstructionTest(q);
}

TEST(Geometry, eulerRpyToQuaternion) {
  Eigen::Quaternionf q;
  Eigen::Vector3f v_euler;

  // Roll
  q = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX());
  v_euler = Eigen::Vector3f{M_PI_2, 0.f, 0.f};
  quaternionsApproxEqualTest(q, px4_ros2::eulerRpyToQuaternion(v_euler), "roll pi/2");
  quaternionsApproxEqualTest(
    q, px4_ros2::eulerRpyToQuaternion(
      v_euler.x(), v_euler.y(),
      v_euler.z()), "roll pi/2");

  // Pitch
  q = Eigen::AngleAxisf(-M_PI / 4.f, Eigen::Vector3f::UnitY());
  v_euler = Eigen::Vector3f{0.f, -M_PI / 4.f, 0.f};
  quaternionsApproxEqualTest(q, px4_ros2::eulerRpyToQuaternion(v_euler), "pitch -pi/4");
  quaternionsApproxEqualTest(
    q, px4_ros2::eulerRpyToQuaternion(
      v_euler.x(), v_euler.y(),
      v_euler.z()), "pitch -pi/4");

  // Yaw
  q = Eigen::AngleAxisf(3.f * M_PI / 4.f, Eigen::Vector3f::UnitZ());
  v_euler = Eigen::Vector3f{0.f, 0.f, 3.f * M_PI / 4.f};
  quaternionsApproxEqualTest(q, px4_ros2::eulerRpyToQuaternion(v_euler), "yaw 3pi/4");
  quaternionsApproxEqualTest(
    q, px4_ros2::eulerRpyToQuaternion(
      v_euler.x(), v_euler.y(),
      v_euler.z()), "yaw 3pi/4");

  // Multi-axis rotations
  q = Eigen::Quaternionf{0.99638f, 0.04736f, 0.05235f, 0.04736f};
  v_euler = Eigen::Vector3f{0.1f, 0.1f, 0.1f};
  quaternionsApproxEqualTest(q, px4_ros2::eulerRpyToQuaternion(v_euler));
  quaternionsApproxEqualTest(
    q, px4_ros2::eulerRpyToQuaternion(
      v_euler.x(), v_euler.y(),
      v_euler.z()));

  q = Eigen::Quaternionf{0.05235f, -0.04736f, 0.05235f, 0.99613f};
  v_euler = Eigen::Vector3f{0.1f, 0.1f, M_PI - 0.1f};
  quaternionsApproxEqualTest(q, px4_ros2::eulerRpyToQuaternion(v_euler));
  quaternionsApproxEqualTest(
    q, px4_ros2::eulerRpyToQuaternion(
      v_euler.x(), v_euler.y(),
      v_euler.z()));

  q = Eigen::Quaternionf{-0.05235f, 0.04736f, 0.05235f, 0.99613f};
  v_euler = Eigen::Vector3f{0.1f, -0.1f, M_PI + 0.1f};
  quaternionsApproxEqualTest(q, px4_ros2::eulerRpyToQuaternion(v_euler));
  quaternionsApproxEqualTest(
    q, px4_ros2::eulerRpyToQuaternion(
      v_euler.x(), v_euler.y(),
      v_euler.z()));

  q = Eigen::Quaternionf{0.99638f, -0.04736f, 0.05235f, -0.04736f};
  v_euler = Eigen::Vector3f{-0.1f, 0.1f, -0.1f};
  quaternionsApproxEqualTest(q, px4_ros2::eulerRpyToQuaternion(v_euler));
  quaternionsApproxEqualTest(
    q, px4_ros2::eulerRpyToQuaternion(
      v_euler.x(), v_euler.y(),
      v_euler.z()));

  q = Eigen::Quaternionf{0.99638f, -0.04736f, -0.05235f, 0.04736f};
  v_euler = Eigen::Vector3f{-0.1f, -0.1f, 0.1f};
  quaternionsApproxEqualTest(q, px4_ros2::eulerRpyToQuaternion(v_euler));
  quaternionsApproxEqualTest(
    q, px4_ros2::eulerRpyToQuaternion(
      v_euler.x(), v_euler.y(),
      v_euler.z()));

  q = Eigen::Quaternionf{0.05235f, 0.04736f, 0.05235f, -0.99613f};
  v_euler = Eigen::Vector3f{-0.1f, 0.1f, -M_PI + 0.1f};
  quaternionsApproxEqualTest(q, px4_ros2::eulerRpyToQuaternion(v_euler));
  quaternionsApproxEqualTest(
    q, px4_ros2::eulerRpyToQuaternion(
      v_euler.x(), v_euler.y(),
      v_euler.z()));

  q = Eigen::Quaternionf{-0.04736f, -0.05235f, -0.04736f, -0.99638f};
  v_euler = Eigen::Vector3f{0.1f, -0.1f, -M_PI - 0.1f};
  quaternionsApproxEqualTest(q, px4_ros2::eulerRpyToQuaternion(v_euler));
  quaternionsApproxEqualTest(
    q, px4_ros2::eulerRpyToQuaternion(
      v_euler.x(), v_euler.y(),
      v_euler.z()));

  q = Eigen::Quaternionf{0.99613f, 0.05235f, 0.04736f, -0.05235f};
  v_euler = Eigen::Vector3f{0.1f, 0.1f, -0.1f};
  quaternionsApproxEqualTest(q, px4_ros2::eulerRpyToQuaternion(v_euler));
  quaternionsApproxEqualTest(
    q, px4_ros2::eulerRpyToQuaternion(
      v_euler.x(), v_euler.y(),
      v_euler.z()));

}

TEST(Geometry, quaternionToRoll) {
  Eigen::Quaternionf q;
  float roll;

  // 90 deg roll
  q = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX());
  roll = M_PI_2;
  EXPECT_NEAR(roll, px4_ros2::quaternionToRoll(q), 1e-3);

  // Rolled -45 deg along Y-axis
  q = Eigen::AngleAxisf(-M_PI / 4.f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());
  roll = -M_PI / 4.f;
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
  q = Eigen::AngleAxisf(-M_PI / 4.f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());
  pitch = 0.f;
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
  q = Eigen::AngleAxisf(-M_PI / 4.f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());
  yaw = M_PI_2;
  EXPECT_NEAR(yaw, px4_ros2::quaternionToYaw(q), 1e-3);
}
