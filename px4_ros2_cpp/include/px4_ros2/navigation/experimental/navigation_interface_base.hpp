/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_ros2/common/context.hpp>

namespace px4_ros2
{

// Define navigation interface return codes
enum class NavigationInterfaceReturnCode : int
{
  SUCCESS = 0,
  ESTIMATE_EMPTY = 1,
  ESTIMATE_VARIANCE_INVALID = 2,
  ESTIMATE_FRAME_UNKNOWN = 3,
  ESTIMATE_VALUE_NAN = 4,
  ESTIMATE_MISSING_TIMESTAMP = 5
};

constexpr inline const char * resultToString(NavigationInterfaceReturnCode result) noexcept
{
  switch (result) {
    case NavigationInterfaceReturnCode::SUCCESS: return "SUCCESS";

    case NavigationInterfaceReturnCode::ESTIMATE_EMPTY: return "ESTIMATE_EMPTY";

    case NavigationInterfaceReturnCode::ESTIMATE_VARIANCE_INVALID: return
        "ESTIMATE_VARIANCE_INVALID";

    case NavigationInterfaceReturnCode::ESTIMATE_FRAME_UNKNOWN: return "ESTIMATE_FRAME_UNKNOWN";

    case NavigationInterfaceReturnCode::ESTIMATE_VALUE_NAN: return "ESTIMATE_VALUE_NAN";

    case NavigationInterfaceReturnCode::ESTIMATE_MISSING_TIMESTAMP: return
        "ESTIMATE_MISSING_TIMESTAMP ";
  }

  return "Unknown";
}

template<typename EstimateType>
class NavigationInterfaceBase : public Context
{
public:
  explicit NavigationInterfaceBase(rclcpp::Node & node)
  : Context(node, ""), _node(node) {}
  virtual ~NavigationInterfaceBase() = default;

  /**
   * @brief Publish position estimate to FMU.
   */
  virtual NavigationInterfaceReturnCode update(const EstimateType & global_position_estimate) const
  = 0;

protected:
  virtual bool _isEstimateNonEmpty(const EstimateType & estimate) const = 0;
  virtual bool _isVarianceValid(const EstimateType & estimate) const = 0;
  virtual bool _isFrameValid(const EstimateType & estimate) const = 0;
  virtual bool _isValueNotNAN(const EstimateType & estimate) const = 0;

  rclcpp::Node & _node;
};

} // namespace px4_ros2
