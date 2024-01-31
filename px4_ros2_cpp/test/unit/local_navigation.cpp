/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

/**
 * Summary:
 * - `LocalPositionInterfaceTest` fixture:
 *   - Set up: LocalPositionMeasurementInterface instance is configured with PoseFrame::LocalNED and VelocityFrame::LocalNED.
 *   - Test Cases:
 *     1. MeasurementEmpty: Sends a measurement with only a timestamp. Expects exception.
 *     2. TimestampMissing: Sends a measurement with missing timestamp. Expects exception.
 *     3. VarianceInvalid: Sends measurements with missing variance fields. Expects exception.
 *     4. ContainsNAN: Sends measurements with NaN values. Expects exception.
 *
 * - `LocalPositionInterfacePoseTest` fixture:
 *   - Set up: LocalPositionMeasurementInterface instance is configured with PoseFrame::LocalNED and VelocityFrame::Unknown.
 *   - Test Cases:
 *     1. PoseFrameUnknown: Sends a measurement with a position, and a measurement with a velocity.
 *        Expects success for position measurements, exception for velocity measurements.
 *
 * - `LocalPositionInterfaceVelocityTest` fixture:
 *   - Set up: LocalPositionMeasurementInterface instance is configured with PoseFrame::Unknown and VelocityFrame::LocalNED.
 *   - Test Cases:
 *     1. VelocityFrameUnknown: Sends a measurement with a position, and a measurement with a velocity.
 *        Expects success for velocity measurements, exception for position measurements.
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>

using px4_ros2::LocalPositionMeasurement, px4_ros2::NavigationInterfaceInvalidArgument;

class LocalPositionInterfaceTestBase : public testing::Test
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
  }

  std::shared_ptr<rclcpp::Node> _node;
  std::shared_ptr<px4_ros2::LocalPositionMeasurementInterface> _local_navigation_interface;
};

// Configure interface with PoseFrame::LocalNED and VelocityFrame::LocalNED
class LocalPositionInterfaceTest : public LocalPositionInterfaceTestBase
{
protected:
  void SetUp() override
  {
    LocalPositionInterfaceTestBase::SetUp();
    _local_navigation_interface = std::make_shared<px4_ros2::LocalPositionMeasurementInterface>(
      *_node,
      px4_ros2::PoseFrame::LocalNED,
      px4_ros2::VelocityFrame::LocalNED);
  }
};

// Configure interface with PoseFrame::LocalNED and VelocityFrame::Unknown
class LocalPositionInterfacePoseTest : public LocalPositionInterfaceTestBase
{
protected:
  void SetUp() override
  {
    LocalPositionInterfaceTestBase::SetUp();
    _local_navigation_interface = std::make_shared<px4_ros2::LocalPositionMeasurementInterface>(
      *_node,
      px4_ros2::PoseFrame::LocalNED,
      px4_ros2::VelocityFrame::Unknown);
  }
};

// Configure interface with PoseFrame::Unknown and VelocityFrame::LocalNED
class LocalPositionInterfaceVelocityTest : public
  LocalPositionInterfaceTestBase
{
protected:
  void SetUp() override
  {
    LocalPositionInterfaceTestBase::SetUp();
    _local_navigation_interface = std::make_shared<px4_ros2::LocalPositionMeasurementInterface>(
      *_node,
      px4_ros2::PoseFrame::Unknown,
      px4_ros2::VelocityFrame::LocalNED);
  }
};

// Tests interface when position measurement is empty
TEST_F(LocalPositionInterfaceTest, MeasurementEmpty) {
  LocalPositionMeasurement measurement{};

  // Send measurement with only a timestamp
  measurement.timestamp_sample = _node->get_clock()->now();
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for empty measurement, but none was thrown.";
}

// Tests interface when measurement timestamp is missing
TEST_F(LocalPositionInterfaceTest, TimestampMissing) {
  LocalPositionMeasurement measurement{};

  measurement.position_xy = Eigen::Vector2f {1.f, 2.f};
  measurement.position_xy_variance = Eigen::Vector2f {0.2f, 0.1f};
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for missing timestamp, but none was thrown.";
}

// Tests interface when a variance is missing
TEST_F(LocalPositionInterfaceTest, VarianceInvalid) {
  LocalPositionMeasurement measurement{};

  // Send position_xy without variance
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.position_xy = Eigen::Vector2f {1.f, 2.f};
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for missing variance (position_xy), but none was thrown.";

  // Send position_z without variance
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.position_z = 12.3f;
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for missing variance (position_z), but none was thrown.";

  // Send velocity_xy without variance
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.velocity_xy = Eigen::Vector2f {1.f, 2.f};
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for missing variance (velocity_xy), but none was thrown.";

  // Send velocity_z without variance
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.velocity_z = 12.3f;
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for missing variance (velocity_z), but none was thrown.";

  // Send attitude without variance
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.attitude_quaternion = Eigen::Quaternionf {0.1f, -0.2f, 0.3f, 0.25F};
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for missing variance (attitude), but none was thrown.";
}

// Tests interface when measurement contains NAN
TEST_F(LocalPositionInterfaceTest, ContainsNAN) {
  LocalPositionMeasurement measurement{};

  // Send position_xy with NAN
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.position_xy = Eigen::Vector2f {NAN, 2.f};
  measurement.position_xy_variance = Eigen::Vector2f {0.2f, 0.1f};
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for NAN value (position_xy), but none was thrown.";

  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.position_xy = Eigen::Vector2f {1.f, 2.f};
  measurement.position_xy_variance = Eigen::Vector2f {NAN, 0.1f};
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for NAN value (position_xy_variance), but none was thrown.";

  // Send position_z with NAN
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.position_z = NAN;
  measurement.position_z_variance = 0.33F;
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for NAN value (position_z), but none was thrown.";

  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.position_z = 12.3f;
  measurement.position_z_variance = NAN;
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for NAN value (position_z_variance), but none was thrown.";

  // Send velocity_xy with NAN
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.velocity_xy = Eigen::Vector2f {NAN, 2.f};
  measurement.velocity_xy_variance = Eigen::Vector2f {0.3f, 0.4f};
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for NAN value (velocity_xy), but none was thrown.";

  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.velocity_xy = Eigen::Vector2f {1.f, 2.f};
  measurement.velocity_xy_variance = Eigen::Vector2f {NAN, 0.4f};
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for NAN value (velocity_xy_variance), but none was thrown.";

  // Send velocity_z with NAN
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.velocity_z = NAN;
  measurement.velocity_z_variance = 0.33F;
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for NAN value (velocity_z), but none was thrown.";

  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.velocity_z = 12.3f;
  measurement.velocity_z_variance = NAN;
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for NAN value (velocity_z_variance), but none was thrown.";

  // Send attitude with NAN
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.attitude_quaternion = Eigen::Quaternionf {NAN, -0.2f, 0.3f, 0.25F};
  measurement.attitude_variance = Eigen::Vector3f {0.2f, 0.1f, 0.05F};
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for NAN value (attitude_quaternion), but none was thrown.";

  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.attitude_quaternion = Eigen::Quaternionf {0.1f, -0.2f, 0.3f, 0.25F};
  measurement.attitude_variance = Eigen::Vector3f {NAN, 0.1f, 0.05F};
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for NAN value (attitude_variance), but none was thrown.";
}

// Tests interface when velocity reference frame is unknown
TEST_F(LocalPositionInterfacePoseTest, PoseFrameUnknown) {
  LocalPositionMeasurement measurement{};

  // Send position_xy and variance
  // Expects success
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.position_xy = Eigen::Vector2f {1.f, 2.f};
  measurement.position_xy_variance = Eigen::Vector2f {0.2f, 0.1f};
  EXPECT_NO_THROW(_local_navigation_interface->update(measurement)) <<
    "Failed to send position update (position_xy) with known pose frame and unknown velocity frame.";

  // Send position_z and variance
  // Expects success
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.position_z = 12.3f;
  measurement.position_z_variance = 0.33F;
  EXPECT_NO_THROW(_local_navigation_interface->update(measurement)) <<
    "Failed to send position update (position_z) with known pose frame and unknown velocity frame.";

  // Send velocity_xy and variance
  // Expects exception
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.velocity_xy = Eigen::Vector2f {1.f, 2.f};
  measurement.velocity_xy_variance = Eigen::Vector2f {0.3f, 0.4f};
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for velocity update (velocity_xy) in unknown velocity frame, but none was thrown.";

  // Send velocity_z and variance
  // Expects exception
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.velocity_z = 12.3f;
  measurement.velocity_z_variance = 0.33F;
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for velocity update (velocity_z) in unknown velocity frame, but none was thrown.";
}

// Tests interface when pose reference frame is unknown
TEST_F(LocalPositionInterfaceVelocityTest, VelocityFrameUnknown) {
  LocalPositionMeasurement measurement{};

  // Send position_xy and variance
  // Expects exception
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.position_xy = Eigen::Vector2f {1.f, 2.f};
  measurement.position_xy_variance = Eigen::Vector2f {0.2f, 0.1f};
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for position update (position_xy) in unknown pose frame, but none was thrown.";

  // Send position_z and variance
  // Expects exception
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.position_z = 12.3f;
  measurement.position_z_variance = 0.33F;
  EXPECT_THROW(
    _local_navigation_interface->update(measurement),
    NavigationInterfaceInvalidArgument) <<
    "Expected exception for position update (position_z) in unknown pose frame, but none was thrown.";

  // Send velocity_xy and variance
  // Expects success
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.velocity_xy = Eigen::Vector2f {1.f, 2.f};
  measurement.velocity_xy_variance = Eigen::Vector2f {0.3f, 0.4f};
  EXPECT_NO_THROW(_local_navigation_interface->update(measurement)) <<
    "Failed to send velocity update (velocity_xy) with known velocity frame and unknown pose frame.";

  // Send velocity_z and variance
  // Expects success
  measurement = {};
  measurement.timestamp_sample = _node->get_clock()->now();
  measurement.velocity_z = 12.3f;
  measurement.velocity_z_variance = 0.33F;
  EXPECT_NO_THROW(_local_navigation_interface->update(measurement)) <<
    "Failed to send velocity update (velocity_z) with known velocity frame and unknown pose frame.";
}
