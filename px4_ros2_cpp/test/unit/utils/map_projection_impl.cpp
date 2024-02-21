/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>
#include <px4_ros2/utils/geometry.hpp>

#include <src/utils/map_projection_impl.hpp>
#include "util.hpp"

class MapProjectionImplTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    _map_projection.initReference(kLatRef, kLonRef, kAltAmslRef, kTimestampRef);
  }

protected:
  static constexpr double kLatRef = 47.3566094;
  static constexpr double kLonRef = 8.5190237;
  static constexpr double kAltAmslRef = 151.3;
  static constexpr uint64_t kTimestampRef = 0;

  px4_ros2::MapProjectionImpl _map_projection;
};


TEST_F(MapProjectionImplTest, isInitialized) {
  ASSERT_TRUE(_map_projection.isInitialized());
}

TEST_F(MapProjectionImplTest, getters) {
  EXPECT_EQ(kLatRef, _map_projection.getProjectionReferenceLat());
  EXPECT_EQ(kLonRef, _map_projection.getProjectionReferenceLon());
  EXPECT_EQ(kAltAmslRef, _map_projection.getProjectionReferenceAlt());
  EXPECT_EQ(kTimestampRef, _map_projection.getProjectionReferenceTimestamp());
  vectorsApproxEqualTest(
    Eigen::Vector3d{kLatRef, kLonRef, kAltAmslRef},
    _map_projection.getProjectionReferencePosition());
}

TEST_F(MapProjectionImplTest, localToGlobal) {

  const Eigen::Vector3f local_position = {0.5f, 1.f, -2.f};
  const Eigen::Vector3d global_position = _map_projection.localToGlobal(local_position);
  const Eigen::Vector3f local_position_new = _map_projection.globalToLocal(global_position);
  const Eigen::Vector3d global_position_new = _map_projection.localToGlobal(local_position_new);

  // Check against expected value
  const Eigen::Vector3d global_position_exp{47.3566138966073, 8.51903697542915, 153.3};
  vectorsApproxEqualTest(global_position_exp, global_position);

  // Check results against inverse operation
  vectorsApproxEqualTest(local_position, local_position_new);
  vectorsApproxEqualTest(global_position, global_position_new);
}

TEST_F(MapProjectionImplTest, globalToLocal) {

  const Eigen::Vector3d global_position = {47.356616973876953, 8.5190505981445313, 163.7};
  const Eigen::Vector3f local_position = _map_projection.globalToLocal(global_position);
  const Eigen::Vector3d global_position_new = _map_projection.localToGlobal(local_position);
  const Eigen::Vector3f local_position_new = _map_projection.globalToLocal(global_position_new);

  // Check against expected value
  const Eigen::Vector3f local_position_exp{0.842f, 2.026f, -12.4f};
  vectorsApproxEqualTest(local_position_exp, local_position);

  // Check results against inverse operation
  vectorsApproxEqualTest(global_position, global_position_new);
  vectorsApproxEqualTest(local_position, local_position_new);
}
