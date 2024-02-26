/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>

namespace px4_ros2
{
/** \ingroup components
 *  @{
 */

/**
 * @brief A ROS2 node which instantiates a control mode and handles its registration with PX4.
 * @tparam ModeT The mode type, which must be derived from px4_ros2::ModeBase.
 *
 * Example usage:
 *
 * @code{.cpp}
 * class MyMode : public px4_ros2::ModeBase {...};
 * rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<MyMode>>("my_node"));
 * @endcode
 *
 * @ingroup components
 */
template<typename ModeT>
class NodeWithMode : public rclcpp::Node
{
  static_assert(
    std::is_base_of<ModeBase, ModeT>::value,
    "Template type ModeT must be derived from px4_ros2::ModeBase");

public:
  explicit NodeWithMode(std::string node_name, bool enable_debug_output = false)
  : Node(node_name)
  {
    if (enable_debug_output) {
      auto ret =
        rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

      if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
      }
    }

    _mode = std::make_unique<ModeT>(*this);

    if (!_mode->doRegister()) {
      throw std::runtime_error("Registration failed");
    }
  }

  ModeT & getMode() const
  {
    return *_mode;
  }

private:
  std::unique_ptr<ModeT> _mode;
};

/**
 * @brief A ROS2 node which instantiates a mode executor and its owned mode, and handles their registration with PX4.
 *
 * Assumes mode executor constructor signature is `ModeExecutorT(rclcpp::Node, ModeT)`.
 *
 * @tparam ModeExecutorT The mode executor type, which must be derived from px4_ros2::ModeExecutorBase.
 * @tparam ModeT The mode type owned by the executor, which must be derived from px4_ros2::ModeBase.
 *
 * Example usage:
 *
 * @code{.cpp}
 * class MyMode : public px4_ros2::ModeBase {...};
 * class MyExecutor : public px4_ros2::ModeExecutor {...};
 * rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<MyExecutor, MyMode>>("my_node"));
 * @endcode
 *
 * @ingroup components
 */
template<typename ModeExecutorT, typename ModeT>
class NodeWithModeExecutor : public rclcpp::Node
{
  static_assert(
    std::is_base_of<ModeExecutorBase, ModeExecutorT>::value,
    "Template type ModeExecutorT must be derived from px4_ros2::ModeExecutorBase");
  static_assert(
    std::is_base_of<ModeBase, ModeT>::value,
    "Template type ModeT must be derived from px4_ros2::ModeBase");

public:
  explicit NodeWithModeExecutor(std::string node_name, bool enable_debug_output = false)
  : Node(node_name)
  {
    if (enable_debug_output) {
      auto ret =
        rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

      if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
      }
    }

    _mode = std::make_unique<ModeT>(*this);
    _mode_executor = std::make_unique<ModeExecutorT>(*this, *_mode);

    if (!_mode_executor->doRegister()) {
      throw std::runtime_error("Registration failed");
    }
  }

  ModeT & getMode() const
  {
    return *_mode;
  }

private:
  std::unique_ptr<ModeExecutorT> _mode_executor;
  std::unique_ptr<ModeT> _mode;
};

/** @}*/
}  // namespace px4_ros2
