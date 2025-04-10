/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once
#include <functional>
#include <memory>
#include <px4_ros2/mission/mission.hpp>

namespace px4_ros2
{
/** \ingroup mission
 *  @{
 */

/**
 * @brief Interface for a trajectory executor
 * @ingroup mission
 */
class TrajectoryExecutorInterface
{
public:
  TrajectoryExecutorInterface() = default;
  TrajectoryExecutorInterface(const TrajectoryExecutorInterface &) = delete;
  virtual ~TrajectoryExecutorInterface() = default;

  virtual bool navigationItemTypeSupported(NavigationItemType type) = 0;
  virtual bool frameSupported(MissionFrame mission_frame) = 0;

  struct TrajectoryConfig
  {
    std::shared_ptr<Mission> trajectory;
    TrajectoryOptions options;
    int start_index{0};
    int end_index{0};
    bool stop_at_last{true};
    std::function<void(int)> on_index_reached;
    std::function<void()> on_failure;
  };
  virtual void runTrajectory(const TrajectoryConfig & config) = 0;

  virtual void updateSetpoint() = 0;
};


/** @}*/
} /* namespace px4_ros2 */
