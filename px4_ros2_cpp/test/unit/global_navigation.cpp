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
#include <px4_ros2/utils/message_version.hpp>

using px4_ros2::GlobalPositionMeasurement, px4_ros2::NavigationInterfaceInvalidArgument;
using namespace std::chrono_literals;

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
    _subscriber =
      _node->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "/fmu/in/aux_global_position" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleGlobalPosition>(), rclcpp::QoS(
        10).best_effort(),
      [this](px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) {
        _update_msg = std::move(msg);
      });
  }

  bool waitForUpdate()
  {
    _update_msg = nullptr;
    auto start_time = _node->get_clock()->now();
    while (!_update_msg) {
      rclcpp::sleep_for(kSleepInterval);
      rclcpp::spin_some(_node);
      const auto elapsed_time = _node->get_clock()->now() - start_time;
      if (elapsed_time >= kTimeoutDuration) {
        return _update_msg != nullptr;
      }
    }
    return true;
  }

  std::shared_ptr<rclcpp::Node> _node;
  std::shared_ptr<px4_ros2::GlobalPositionMeasurementInterface> _global_navigation_interface;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr _subscriber;
  px4_msgs::msg::VehicleGlobalPosition::UniquePtr _update_msg;

  static constexpr std::chrono::seconds kTimeoutDuration{3s};
  static constexpr std::chrono::milliseconds kSleepInterval{10ms};

  // Test measurements
  static constexpr double kLat{-33.925937};
  static constexpr double kLon{18.427745};
  static constexpr float kHorizontalVariance{0.5F};
};


// Tests interface when position measurement is valid
TEST_F(GlobalPositionInterfaceTest, Valid) {

  // Send measurement
  GlobalPositionMeasurement measurement{};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.lat_lon = Eigen::Vector2d {kLat, kLon};
  measurement.horizontal_variance = kHorizontalVariance;

  ASSERT_NO_THROW(_global_navigation_interface->update(measurement));

  // Wait for the ROS2 update message
  ASSERT_TRUE(waitForUpdate());
  EXPECT_DOUBLE_EQ(_update_msg->lat, kLat);
  EXPECT_DOUBLE_EQ(_update_msg->lon, kLon);
  EXPECT_NEAR(
    _update_msg->eph, std::sqrt(kHorizontalVariance), std::sqrt(
      kHorizontalVariance) * 0.05);
  EXPECT_EQ(_update_msg->lat_lon_reset_counter, 0);

  // Signal measurement reset and send new measurement
  _global_navigation_interface->reset();
  ASSERT_NO_THROW(_global_navigation_interface->update(measurement));

  ASSERT_TRUE(waitForUpdate());
  EXPECT_EQ(_update_msg->lat_lon_reset_counter, 1);
}

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
