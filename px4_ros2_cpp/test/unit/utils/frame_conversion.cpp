/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "util.hpp"

#include <gtest/gtest.h>
#include <px4_ros2/utils/frame_conversion.hpp>


TEST(FrameConversion, yawNedToEnu) {
  EXPECT_FLOAT_EQ(M_PI_2, px4_ros2::yawNedToEnu(0.f));
  EXPECT_FLOAT_EQ(M_PI / 4.f, px4_ros2::yawNedToEnu(M_PI / 4.f));
  EXPECT_FLOAT_EQ(-M_PI_2, px4_ros2::yawNedToEnu(M_PI));
  EXPECT_FLOAT_EQ(M_PI, std::fabs(px4_ros2::yawNedToEnu(-M_PI_2)));
}

TEST(FrameConversion, yawEnuToNed) {
  EXPECT_FLOAT_EQ(M_PI_2, px4_ros2::yawEnuToNed(0.f));
  EXPECT_FLOAT_EQ(M_PI / 4.f, px4_ros2::yawEnuToNed(M_PI / 4.f));
  EXPECT_FLOAT_EQ(-M_PI_2, px4_ros2::yawEnuToNed(M_PI));
  EXPECT_FLOAT_EQ(M_PI, std::fabs(px4_ros2::yawEnuToNed(-M_PI_2)));
}

TEST(FrameConversion, yawRateNedToEnu) {
  EXPECT_FLOAT_EQ(0.f, px4_ros2::yawRateNedToEnu(0.f));
  EXPECT_FLOAT_EQ(12.3f, px4_ros2::yawRateNedToEnu(-12.3f));
}

TEST(FrameConversion, yawRateEnuToNed) {
  EXPECT_FLOAT_EQ(0.f, px4_ros2::yawRateEnuToNed(0.f));
  EXPECT_FLOAT_EQ(12.3f, px4_ros2::yawRateEnuToNed(-12.3f));
}

TEST(FrameConversion, attitudeNedToEnu) {
  Eigen::Quaternionf q_ned;
  Eigen::Quaternionf q_enu;

  // North
  q_ned = Eigen::Quaternionf::Identity();
  q_enu = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_enu, px4_ros2::attitudeNedToEnu(q_ned));

  // Test various multi-axis rotations
  q_ned = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitZ());
  q_enu = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_enu, px4_ros2::attitudeNedToEnu(q_ned));

  q_ned = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(M_PI - 0.1f, Eigen::Vector3f::UnitZ());
  q_enu = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-M_PI + 0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_enu, px4_ros2::attitudeNedToEnu(q_ned));

  q_ned = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(M_PI + 0.1f, Eigen::Vector3f::UnitZ());
  q_enu = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-M_PI - 0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_enu, px4_ros2::attitudeNedToEnu(q_ned));

  q_ned = Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitZ());
  q_enu = Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_enu, px4_ros2::attitudeNedToEnu(q_ned));

  q_ned = Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitZ());
  q_enu = Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_enu, px4_ros2::attitudeNedToEnu(q_ned));

  q_ned = Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-M_PI + 0.1f, Eigen::Vector3f::UnitZ());
  q_enu = Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(M_PI - 0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_enu, px4_ros2::attitudeNedToEnu(q_ned));

  q_ned = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-M_PI - 0.1f, Eigen::Vector3f::UnitZ());
  q_enu = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(M_PI + 0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_enu, px4_ros2::attitudeNedToEnu(q_ned));

  q_ned = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitZ());
  q_enu = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_enu, px4_ros2::attitudeNedToEnu(q_ned));
}

TEST(FrameConversion, attitudeEnuToNed) {
  Eigen::Quaternionf q_ned;
  Eigen::Quaternionf q_enu;

  // East
  q_enu = Eigen::Quaternionf::Identity();
  q_ned = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_enu, px4_ros2::attitudeEnuToNed(q_ned));

  // Test various multi-axis rotations
  q_enu = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitZ());
  q_ned = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_ned, px4_ros2::attitudeEnuToNed(q_enu));

  q_enu = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(M_PI - 0.1f, Eigen::Vector3f::UnitZ());
  q_ned = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-M_PI + 0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_ned, px4_ros2::attitudeEnuToNed(q_enu));

  q_enu = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(M_PI + 0.1f, Eigen::Vector3f::UnitZ());
  q_ned = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-M_PI - 0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_ned, px4_ros2::attitudeEnuToNed(q_enu));

  q_enu = Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitZ());
  q_ned = Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_ned, px4_ros2::attitudeEnuToNed(q_enu));

  q_enu = Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitZ());
  q_ned = Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_ned, px4_ros2::attitudeEnuToNed(q_enu));

  q_enu = Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-M_PI + 0.1f, Eigen::Vector3f::UnitZ());
  q_ned = Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(M_PI - 0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_ned, px4_ros2::attitudeEnuToNed(q_enu));

  q_enu = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-M_PI - 0.1f, Eigen::Vector3f::UnitZ());
  q_ned = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(M_PI + 0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_ned, px4_ros2::attitudeEnuToNed(q_enu));

  q_enu = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitZ());
  q_ned = Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.1f + M_PI_2, Eigen::Vector3f::UnitZ());
  quaternionsApproxEqualTest(q_ned, px4_ros2::attitudeEnuToNed(q_enu));
}

TEST(FrameConversion, positionNedToEnu) {
  Eigen::Vector3f v_ned(1.f, 2.f, 3.f);
  Eigen::Vector3f v_enu(2.f, 1.f, -3.f);
  vectorsApproxEqualTest(v_enu, px4_ros2::positionNedToEnu(v_ned));
}

TEST(FrameConversion, positionEnuToNed) {
  Eigen::Vector3f v_enu(1.f, 2.f, 3.f);
  Eigen::Vector3f v_ned(2.f, 1.f, -3.f);
  vectorsApproxEqualTest(v_ned, px4_ros2::positionEnuToNed(v_enu));
}

TEST(FrameConversion, frdToFlu) {
  Eigen::Vector3f v_frd(1.f, 2.f, 3.f);
  Eigen::Vector3f v_flu(1.f, -2.f, -3.f);
  vectorsApproxEqualTest(v_flu, px4_ros2::frdToFlu(v_frd));
}

TEST(FrameConversion, fluToFrd) {
  Eigen::Vector3f v_flu(1.f, 2.f, 3.f);
  Eigen::Vector3f v_frd(1.f, -2.f, -3.f);
  vectorsApproxEqualTest(v_frd, px4_ros2::fluToFrd(v_flu));
}

TEST(FrameConversion, varianceNedToEnu) {
  Eigen::Vector3f v_enu(1.f, 2.f, 3.f);
  Eigen::Vector3f v_ned(2.f, 1.f, 3.f);
  vectorsApproxEqualTest(v_ned, px4_ros2::varianceNedToEnu(v_enu));
}

TEST(FrameConversion, varianceEnuToNed) {
  Eigen::Vector3f v_enu(1.f, 2.f, 3.f);
  Eigen::Vector3f v_ned(2.f, 1.f, 3.f);
  vectorsApproxEqualTest(v_ned, px4_ros2::varianceEnuToNed(v_enu));
}

TEST(FrameConversion, yawBodyToWorld) {
  float yaw;
  Eigen::Vector3f point_body;
  Eigen::Vector3f point_world;

  yaw = 0.f;
  point_body = Eigen::Vector3f(1.f, 2.f, 3.f);
  vectorsApproxEqualTest(point_body, px4_ros2::yawBodyToWorld(yaw, point_body));

  yaw = M_PI_2;
  point_body = Eigen::Vector3f(1.f, 2.f, 3.f);
  point_world = Eigen::Vector3f(-2.f, 1.f, 3.f);
  vectorsApproxEqualTest(point_world, px4_ros2::yawBodyToWorld(yaw, point_body));

  yaw = -3.f * M_PI / 4.f;
  point_body = Eigen::Vector3f(1.f, 2.f, 3.f);
  point_world = Eigen::Vector3f(std::sqrt(2) / 2, -3 / std::sqrt(2), 3.f);
  vectorsApproxEqualTest(point_world, px4_ros2::yawBodyToWorld(yaw, point_body));

  yaw = -M_PI_2 + 0.1f;
  point_body = Eigen::Vector3f(1.f, 2.f, 3.f);
  point_world = Eigen::Vector3f(
    std::cos(yaw) * point_body.x() - std::sin(yaw) * point_body.y(),
    std::sin(yaw) * point_body.x() + std::cos(yaw) * point_body.y(),
    3.f
  );
  vectorsApproxEqualTest(point_world, px4_ros2::yawBodyToWorld(yaw, point_body));
}
