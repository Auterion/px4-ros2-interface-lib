/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <tuple>
#include <utility>
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
 * Assumes mode executor constructor signature is `ModeExecutorT(rclcpp::Node, OwnedModeT, OtherModesT...)`.
 *
 * @tparam ModeExecutorT The mode executor type, which must be derived from px4_ros2::ModeExecutorBase.
 * @tparam OwnedModeT The mode type owned by the executor, which must be derived from px4_ros2::ModeBase.
 * @tparam OtherModesT Optional additional modes not owned by the executor but registered by the node, which must be derived from px4_ros2::ModeBase.
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
template<typename ModeExecutorT, typename OwnedModeT, typename ... OtherModesT>
class NodeWithModeExecutor : public rclcpp::Node
{
  static_assert(
    std::is_base_of_v<ModeExecutorBase, ModeExecutorT>,
    "Template type ModeExecutorT must be derived from px4_ros2::ModeExecutorBase");
  static_assert(
    (std::is_base_of_v<ModeBase, OwnedModeT>&& ... && std::is_base_of_v<ModeBase, OtherModesT>),
    "Template types OwnedModeT and OtherModesT must be derived from px4_ros2::ModeBase");

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

    _owned_mode = std::make_unique<OwnedModeT>(*this);
    _other_modes = std::make_tuple(std::make_unique<OtherModesT>(*this)...);
    _mode_executor = createModeExecutor(std::index_sequence_for<OtherModesT...>{});

    if (!_mode_executor->doRegister() || !std::apply(
        [](const auto &... mode) {
          // *INDENT-OFF*
          return (mode->doRegister() && ...);
          // *INDENT-ON*
        }, _other_modes))
    {
      throw std::runtime_error("Registration failed");
    }
  }

  template<typename ModeT = OwnedModeT>
  ModeT & getMode() const
  {
    if constexpr (std::is_same_v<ModeT, OwnedModeT>) {
      return *_owned_mode;
    } else {
      return *std::get<ModeT>(_other_modes);
    }
  }

private:
  template<std::size_t... Idx>
  auto createModeExecutor(std::index_sequence<Idx...>)
  {
    if constexpr (sizeof...(Idx) == 0) {
      return std::make_unique<ModeExecutorT>(*this, *_owned_mode);
    } else {
      return std::make_unique<ModeExecutorT>(*this, *_owned_mode, *std::get<Idx>(_other_modes)...);
    }
  }

  std::unique_ptr<ModeExecutorT> _mode_executor;
  std::unique_ptr<OwnedModeT> _owned_mode;
  std::tuple<std::unique_ptr<OtherModesT>...> _other_modes;
};

/** @}*/
}  // namespace px4_ros2
