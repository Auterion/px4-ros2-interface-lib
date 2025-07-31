/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

/**
 * Summary:
 *  Tests that measurements sent via the interface are fused into PX4.
 * - `GlobalPositionInterfaceTest` fixture:
 *   - Test Cases:
 *     1. FuseAll: Sends measurements with all fields set. Expects cs_aux_gpos to be set to true.
 */


#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <px4_ros2/navigation/experimental/global_position_measurement_interface.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include "util.hpp"

using namespace std::chrono_literals;
using px4_ros2::GlobalPositionMeasurement, px4_ros2::GlobalPositionMeasurementInterface,
px4_ros2::OdometryGlobalPosition;
using px4_msgs::msg::EstimatorStatusFlags;

class GlobalPositionInterfaceTest : public BaseTest
{
protected:
  void SetUp() override
  {
    _node = initNode();

    _global_navigation_interface = std::make_shared<GlobalPositionMeasurementInterface>(*_node);
    _position_interface = std::make_shared<OdometryGlobalPosition>(*_global_navigation_interface);

    ASSERT_TRUE(_global_navigation_interface->doRegister()) <<
      "Failed to register GlobalPositionMeasurementInterface.";

    // Subscribe to PX4 EKF estimator status flags
    _subscriber = _node->create_subscription<EstimatorStatusFlags>(
      "fmu/out/estimator_status_flags" + px4_ros2::getMessageNameVersion<EstimatorStatusFlags>(),
      rclcpp::QoS(
        10).best_effort(),
      [this](EstimatorStatusFlags::UniquePtr msg) {
        _estimator_status_flags = std::move(msg);
      });
  }

  void waitUntilFlagsReset()
  {
    auto start_time = _node->now();

    // Wait for PX4 to stop fusing external measurements
    while (_estimator_status_flags->cs_aux_gpos) {
      rclcpp::spin_some(_node);
      rclcpp::sleep_for(kSleepInterval);

      const auto elapsed_time = _node->now() - start_time;

      if (elapsed_time >= kTimeoutDuration) {
        FAIL() << "Timeout while waiting for PX4 estimator flags to reset.";
        break;
      }
    }

    _estimator_status_flags.reset();
  }

  void waitForMeasurementUpdate(
    std::unique_ptr<GlobalPositionMeasurement> measurement,
    const std::function<bool(const EstimatorStatusFlags::UniquePtr &)> & is_fused_getter
  )
  {
    auto start_time = _node->now();

    // Wait for PX4 estimator flags to confirm proper measurement fusion
    while (!_estimator_status_flags || !is_fused_getter(_estimator_status_flags)) {
      // Check timeout
      const auto elapsed_time = _node->now() - start_time;
      if (elapsed_time >= kTimeoutDuration) {
        ASSERT_NE(_estimator_status_flags, nullptr) <<
          "Missing feedback from PX4: no estimator status flags published over fmu/out/estimator_status_flags.";
        EXPECT_TRUE(is_fused_getter(_estimator_status_flags)) <<
          "Position measurement update was not fused into the PX4 EKF.";
        break;
      }

      // Send measurement
      measurement->timestamp_sample = _node->get_clock()->now();
      ASSERT_NO_THROW(
        _global_navigation_interface->update(*measurement)
      ) << "Failed to send position measurement update via GlobalPositionMeasurementInterface.";

      rclcpp::sleep_for(kSleepInterval);
      rclcpp::spin_some(_node);
    }

    waitUntilFlagsReset();
  }

  std::shared_ptr<rclcpp::Node> _node;
  std::shared_ptr<GlobalPositionMeasurementInterface> _global_navigation_interface;
  std::shared_ptr<OdometryGlobalPosition> _position_interface;
  rclcpp::Subscription<EstimatorStatusFlags>::SharedPtr _subscriber;
  EstimatorStatusFlags::UniquePtr _estimator_status_flags;

  static constexpr std::chrono::seconds kTimeoutDuration = 30s;
  static constexpr std::chrono::milliseconds kSleepInterval = 50ms;
};

TEST_F(GlobalPositionInterfaceTest, fuseAll) {
  auto measurement = std::make_unique<GlobalPositionMeasurement>();

  // Wait for the vehicle position
  auto start_time = _node->now();
  while (!_position_interface->positionValid()) {
    // Check timeout
    if (_node->now() - start_time >= kTimeoutDuration) {
      FAIL() << "Timeout while waiting for position to be valid.";
      break;
    }

    rclcpp::spin_some(_node);
    rclcpp::sleep_for(kSleepInterval);
  }
  const auto position = _position_interface->position();

  measurement->lat_lon = Eigen::Vector2d {position.x(), position.y()};
  measurement->horizontal_variance = 0.01F;
  measurement->altitude_msl = position.z();
  measurement->vertical_variance = 0.01F;
  waitForMeasurementUpdate(
    std::move(measurement), [](const EstimatorStatusFlags::UniquePtr & flags) {
      return flags->cs_aux_gpos;
    });
}
