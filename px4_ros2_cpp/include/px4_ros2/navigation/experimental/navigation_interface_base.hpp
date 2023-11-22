/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_ros2/common/context.hpp>
#include <px4_ros2/navigation/experimental/navigation_interface_common.hpp>

namespace px4_ros2
{

template<typename EstimateType>
class NavigationInterfaceBase
{
public:
  explicit NavigationInterfaceBase() = default;
  virtual ~NavigationInterfaceBase() = default;

  /**
   * @brief Publish position estimate to FMU.
   */
  virtual NavigationInterfaceReturnCode update(const EstimateType & global_position_estimate) const
  = 0;

private:
  virtual bool _checkEstimateEmpty(const EstimateType & estimate) const = 0;
  virtual bool _checkVarianceValid(const EstimateType & estimate) const = 0;
  virtual bool _checkFrameValid(const EstimateType & estimate) const = 0;
  virtual bool _checkValuesNotNAN(const EstimateType & estimate) const = 0;
};

} // namespace px4_ros2
