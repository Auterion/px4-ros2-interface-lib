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
  Success = 0,
  EstimateEmpty = 1,
  EstimateVarianceInvalid = 2,
  EstimateFrameUnknown = 3,
  EstimateValueNan = 4,
  EstimateMissingTimestamp = 5
};

constexpr inline const char * resultToString(NavigationInterfaceReturnCode result) noexcept
{
  switch (result) {
    case NavigationInterfaceReturnCode::Success: return "Success";

    case NavigationInterfaceReturnCode::EstimateEmpty: return "EstimateEmpty";

    case NavigationInterfaceReturnCode::EstimateVarianceInvalid: return
        "EstimateVarianceInvalid";

    case NavigationInterfaceReturnCode::EstimateFrameUnknown: return "EstimateFrameUnknown";

    case NavigationInterfaceReturnCode::EstimateValueNan: return "EstimateValueNan";

    case NavigationInterfaceReturnCode::EstimateMissingTimestamp: return
        "EstimateMissingTimestamp ";
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
  virtual bool isEstimateNonEmpty(const EstimateType & estimate) const = 0;
  virtual bool isVarianceValid(const EstimateType & estimate) const = 0;
  virtual bool isFrameValid(const EstimateType & estimate) const = 0;
  virtual bool isValueNotNAN(const EstimateType & estimate) const = 0;

  rclcpp::Node & _node;
};

} // namespace px4_ros2
