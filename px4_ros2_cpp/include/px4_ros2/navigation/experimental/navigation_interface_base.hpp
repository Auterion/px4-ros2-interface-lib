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

class NavigationInterfaceBase : public Context
{
public:
  explicit NavigationInterfaceBase(rclcpp::Node & node, std::string topic_namespace_prefix = "")
  : Context(node, std::move(topic_namespace_prefix)), _node(node) {}
  virtual ~NavigationInterfaceBase() = default;

  /**
   * Register the interface.
   * @return true on success
   */
  bool doRegister()
  {
    return true;
  }

protected:
  rclcpp::Node & _node;
};

} // namespace px4_ros2
