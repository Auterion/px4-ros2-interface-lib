/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>
#include <px4_ros2/utils/geodesic.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include "util.hpp"

using namespace px4_ros2::literals; // NOLINT

/**
 * Expected computation results from https://www.movable-type.co.uk/scripts/latlong.html
*/

TEST(Geodesic, horizontalDistanceToGlobalPosition) {
  const Eigen::Vector3d global_position_start{47.3566094, 8.5190237, 151.3};
  const Eigen::Vector3d global_position_end{47.35839, 8.52894, 581.3};
  const float distance_exp = 772.749f;

  // Check against expected value
  const float distance_res = px4_ros2::horizontalDistanceToGlobalPosition(
    global_position_start,
    global_position_end);
  EXPECT_NEAR(distance_exp, distance_res, 1e-3);
}

TEST(Geodesic, distanceToGlobalPosition) {
  const Eigen::Vector3d global_position_start{47.3566094, 8.5190237, 151.3};
  const Eigen::Vector3d global_position_end{47.35839, 8.52894, 581.3};
  const float distance_exp = 884.331f;

  // Check against expected value
  const float distance_res = px4_ros2::distanceToGlobalPosition(
    global_position_start,
    global_position_end);
  EXPECT_NEAR(distance_exp, distance_res, 1e-3);
}

TEST(Geodesic, headingToGlobalPosition) {
  const Eigen::Vector3d global_position_start{47.3566094, 8.5190237, 151.3};
  const Eigen::Vector3d global_position_end{47.35839, 8.52894, 581.3};
  const float heading_exp = 75.1505_deg;

  // Check against expected value
  const float heading_res = px4_ros2::headingToGlobalPosition(
    global_position_start,
    global_position_end);
  EXPECT_NEAR(heading_exp, heading_res, 1e-3);
}

TEST(Geodesic, vectorToGlobalPosition) {
  const Eigen::Vector3d global_position_start{47.3566094, 8.5190237, 151.3};
  const Eigen::Vector3d global_position_end{47.35839, 8.52894, 581.3};
  const Eigen::Vector3f vector_exp{198.041f, 746.941f, -430.f};

  // Check against expected value
  const Eigen::Vector3f vector_res = px4_ros2::vectorToGlobalPosition(
    global_position_start,
    global_position_end);
  vectorsApproxEqualTest(vector_exp, vector_res);

  // Check resulting vector matches initial start -> end position displacement
  const Eigen::Vector3d global_position_end_new = px4_ros2::addVectorToGlobalPosition(
    global_position_start, vector_res);
  vectorsApproxEqualTest(global_position_end, global_position_end_new);
}

TEST(Geodesic, globalPositionFromLineAndDist) {
  const Eigen::Vector2d global_position_start{47.3566094, 8.5190237};
  const Eigen::Vector2d global_position_end{47.35839, 8.52894};
  const float dist_from_start = 123.4f;
  const Eigen::Vector2d global_position_exp{47.35689, 8.52061};

  // Check against expected value
  const Eigen::Vector2d global_position_res = px4_ros2::globalPositionFromLineAndDist(
    global_position_start, global_position_end, dist_from_start);
  EXPECT_TRUE(global_position_exp.isApprox(global_position_res, 1e-3));

  // Check resulting position distance from start position matches provided distance
  const float dist_new = px4_ros2::horizontalDistanceToGlobalPosition(
    global_position_start,
    global_position_res);
  EXPECT_NEAR(dist_from_start, dist_new, 1e-3);
}

TEST(Geodesic, globalPositionFromHeadingAndDist) {
  const Eigen::Vector3d global_position_start{47.3566094, 8.5190237, 151.3};
  const float heading = 19.4_deg;
  const float dist = 123.4f;
  const Eigen::Vector3d global_position_exp{47.3577, 8.5196, 151.3};

  // Check against expected value
  const Eigen::Vector3d global_position_res = px4_ros2::globalPositionFromHeadingAndDist(
    global_position_start, heading, dist);
  vectorsApproxEqualTest(global_position_exp, global_position_res);

  // Check resulting position heading and distance from start position match provided values
  const float heading_new = px4_ros2::headingToGlobalPosition(
    global_position_start,
    global_position_res);
  const float dist_new = px4_ros2::distanceToGlobalPosition(
    global_position_start,
    global_position_res);
  EXPECT_NEAR(heading, heading_new, 1e-3);
  EXPECT_NEAR(dist, dist_new, 1e-3);
}

TEST(Geodesic, addVectorToGlobalPosition) {
  const Eigen::Vector3d global_position{47.3566094, 8.5190237, 151.3};
  const Eigen::Vector3f vector{198.041f, 746.941f, -430.f};
  const Eigen::Vector3d global_position_exp{47.35839, 8.52894, 581.3};

  // Check against expected value
  const Eigen::Vector3d global_position_res = px4_ros2::addVectorToGlobalPosition(
    global_position,
    vector);
  vectorsApproxEqualTest(global_position_exp, global_position_res);

  // Check resulting position displacement from the start position matches the provided vector
  const Eigen::Vector3f vector_new = px4_ros2::vectorToGlobalPosition(
    global_position,
    global_position_res);
  vectorsApproxEqualTest(vector, vector_new);
}
