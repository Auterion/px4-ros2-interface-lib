/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <gtest/gtest.h>

using namespace std::chrono_literals; // NOLINT

class RosLogCapture
{
public:
  explicit RosLogCapture(const std::shared_ptr<rclcpp::Node> & node)
  : _node(node)
  {
    // Enable debug output
    const auto log_ret =
      rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (log_ret != RCUTILS_RET_OK) {
      rcutils_reset_error();
    }
    _log_sub = _node->create_subscription<
      rcl_interfaces::msg::Log>(
      "/rosout", rclcpp::QoS(100),
      [this](const rcl_interfaces::msg::Log & msg)
      {
        if (msg.name == _node->get_name()) {
          _captured_lines.push_back(msg.msg);
          if (msg.level >= RCUTILS_LOG_SEVERITY_ERROR) {
            _captured_errors.push_back(msg.msg);
          }
        }
      });
    // Output stdout to stderr. This results in correctly interleaved output as gtest outputs to stderr whereas
    // ros log go to stderr
    dup2(1, 2);
  }

  void expectEqual(std::string expected) const
  {
    // Trim
    while (!expected.empty() && expected[0] == '\n') {
      expected = expected.substr(1);
    }
    while (!expected.empty() && expected[expected.length() - 1] == '\n') {
      expected.pop_back();
    }
    expected += '\n';

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(_node);


    // The full output might not have been captured yet. Count the lines and wait if needed
    const unsigned expected_num_lines =
      static_cast<unsigned>(std::count(expected.begin(), expected.end(), '\n'));
    const auto start = _node->get_clock()->now();
    while (expected_num_lines > _captured_lines.size() ||
      _node->get_clock()->now() - start < 50ms)
    {
      executor.spin_some();
      std::this_thread::yield();
      if (_node->get_clock()->now() - start > 1s) {
        break;
      }
    }

    std::string captured_output;
    for (const auto & line : _captured_lines) {
      captured_output += line + "\n";
    }
    if (expected != captured_output) {
      printf("--- Captured output: ---\n%s--- End ---\n", captured_output.c_str());
    }
    EXPECT_EQ(expected, captured_output);
  }

  void checkHasNoErrors() const
  {
    if (!_captured_errors.empty()) {
      std::string errors;
      for (const auto & error : _captured_errors) {
        errors += error + '\n';
      }
      EXPECT_TRUE(false) << "Captured output contains errors: " << errors;
    }

  }

private:
  std::vector<std::string> _captured_lines;
  std::vector<std::string> _captured_errors;
  std::shared_ptr<rclcpp::Node> _node;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr _log_sub;
};
