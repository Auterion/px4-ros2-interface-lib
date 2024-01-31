/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

/**
 * Summary:
 * - `GlobalPositionInterfaceTest` fixture:
 *   - Test Cases:
 *     1. MeasurementEmpty: Sends a measurement with only a timestamp. Expects exception.
 *     2. TimestampMissing: Sends a measurement with missing timestamp. Expects exception.
 *     3. VarianceInvalid: Sends measurements with missing variance fields. Expects exception.
 *     4. ContainsNAN: Sends measurements with NaN values. Expects exception.
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/global_position_measurement_interface.hpp>

using px4_ros2::GlobalPositionMeasurement, px4_ros2::NavigationInterfaceInvalidArgument;


class GlobalPositionInterfaceTest : public testing::Test
{
protected:
  void SetUp() override
  {
    _node = std::make_shared<rclcpp::Node>("test_node");

    // Suppress logging for exception handling tests
    auto ret = rcutils_logging_set_logger_level(
      _node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_FATAL);
    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(
        _node->get_logger(), "Error setting severity: %s",
        rcutils_get_error_string().str);
      rcutils_reset_error();
    }

    _global_navigation_interface = std::make_shared<px4_ros2::GlobalPositionMeasurementInterface>(
      *_node);
  }

  std::shared_ptr<rclcpp::Node> _node;
  std::shared_ptr<px4_ros2::GlobalPositionMeasurementInterface> _global_navigation_interface;
};

// Tests interface when position measurement is empty
TEST_F(GlobalPositionInterfaceTest, MeasurementEmpty) {
  GlobalPositionMeasurement measurement{};

  // Send measurement with only a timestamp
  measurement.timestamp_sample = _node->get_clock()->now();
  EXPECT_THROW(
    _global_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for empty measurement, but none was thrown.";
}

// Tests interface when measurement timestamp is missing
TEST_F(GlobalPositionInterfaceTest, TimestampMissing) {
  GlobalPositionMeasurement measurement{};

  measurement.lat_lon = Eigen::Vector2d {12.34567, 23.45678};
  measurement.horizontal_variance = 0.1f;
  EXPECT_THROW(
    _global_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for missing timestamp, but none was thrown.";
}

// Tests interface when a variance is missing
TEST_F(GlobalPositionInterfaceTest, VarianceInvalid) {
  GlobalPositionMeasurement measurement{};

  // Send lat lon without variance
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.lat_lon = Eigen::Vector2d {12.34567, 23.45678};
  EXPECT_THROW(
    _global_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for missing variance (lat_lon), but none was thrown.";

  // Send altitude without variance
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.altitude_msl = 123.f;
  EXPECT_THROW(
    _global_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for missing variance (altitude), but none was thrown.";
}

// Tests interface when measurement contains NAN
TEST_F(GlobalPositionInterfaceTest, ContainsNAN) {
  GlobalPositionMeasurement measurement{};

  // Send lat lon with NAN
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.lat_lon = Eigen::Vector2d {NAN, 23.45678};
  measurement.horizontal_variance = 0.1f;
  EXPECT_THROW(
    _global_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for NAN value (lat_lon), but none was thrown.";

  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.lat_lon = Eigen::Vector2d {12.34567, 23.45678};
  measurement.horizontal_variance = NAN;
  EXPECT_THROW(
    _global_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for NAN value (horizontal_variance), but none was thrown.";

  // Send altitude with NAN
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.altitude_msl = NAN;
  measurement.vertical_variance = 0.1f;
  EXPECT_THROW(
    _global_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for NAN value (altitude), but none was thrown.";

  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.altitude_msl = 123.f;
  measurement.vertical_variance = NAN;
  EXPECT_THROW(
    _global_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for NAN value (vertical_variance), but none was thrown.";
}
