/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

/**
 * Summary:
 *  Tests that measurements sent via the interface are fused into PX4.
 * - `LocalPositionInterfaceTest` fixture:
 *   - Set up: LocalPositionMeasurementInterface instance is configured with PoseFrame::LocalNED and VelocityFrame::LocalNED.
 *   - Test Cases:
 *     1. FuseEvPos: Sends position measurements, expects cs_ev_pos and cs_ev_hgt flags to be set to true.
 *     2. FuseEvVel: Sends velocity measurements, expects cs_ev_vel flag to be set to true.
 *     3. FuseEvYaw: Sends attitude measurements, expects cs_ev_yaw flag to be set to true.
 *     4. FuseAll: Sends measurements with all fields set. Expect all flags above to be true.
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include "util.hpp"

using namespace std::chrono_literals;
using px4_ros2::LocalPositionMeasurement, px4_ros2::LocalPositionMeasurementInterface;
using px4_msgs::msg::EstimatorStatusFlags;

class LocalPositionInterfaceTest : public BaseTest
{
protected:
  void SetUp() override
  {
    _node = initNode();

    _local_navigation_interface = std::make_shared<LocalPositionMeasurementInterface>(
      *_node, px4_ros2::PoseFrame::LocalNED,
      px4_ros2::VelocityFrame::LocalNED);
    ASSERT_TRUE(_local_navigation_interface->doRegister()) <<
      "Failed to register LocalPositionMeasurementInterface.";

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
    while (_estimator_status_flags->cs_ev_pos || _estimator_status_flags->cs_ev_hgt ||
      _estimator_status_flags->cs_ev_vel || _estimator_status_flags->cs_ev_yaw)
    {
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
    std::unique_ptr<LocalPositionMeasurement> measurement,
    const std::function<bool(const EstimatorStatusFlags::UniquePtr &)> & is_fused_getter
  )
  {
    auto start_time = _node->now();

    // Wait for PX4 estimator flags to confirm proper measurement fusion
    while (!_estimator_status_flags || !is_fused_getter(_estimator_status_flags)) {

      // Send measurement
      measurement->timestamp_sample = _node->get_clock()->now();
      ASSERT_NO_THROW(
        _local_navigation_interface->update(*measurement)
      ) << "Failed to send position measurement update via LocalPositionMeasurementInterface.";

      rclcpp::spin_some(_node);

      // Check timeout
      const auto elapsed_time = _node->now() - start_time;

      if (elapsed_time >= kTimeoutDuration) {
        ASSERT_NE(_estimator_status_flags, nullptr) <<
          "Missing feedback from PX4: no estimator status flags published over fmu/out/estimator_status_flags.";
        EXPECT_TRUE(is_fused_getter(_estimator_status_flags)) <<
          "Position measurement update was not fused into the PX4 EKF.";
        break;
      }

      rclcpp::sleep_for(kSleepInterval);
    }

    waitUntilFlagsReset();
  }

  std::shared_ptr<rclcpp::Node> _node;
  std::shared_ptr<LocalPositionMeasurementInterface> _local_navigation_interface;
  rclcpp::Subscription<EstimatorStatusFlags>::SharedPtr _subscriber;
  EstimatorStatusFlags::UniquePtr _estimator_status_flags;

  static constexpr std::chrono::seconds kTimeoutDuration = 60s;
  static constexpr std::chrono::milliseconds kSleepInterval = 50ms;
};

TEST_F(LocalPositionInterfaceTest, fuseEvPos) {
  auto measurement = std::make_unique<LocalPositionMeasurement>();
  measurement->position_xy = Eigen::Vector2f {0.f, 0.f};
  measurement->position_xy_variance = Eigen::Vector2f {0.1f, 0.1f};
  measurement->position_z = 0.f;
  measurement->position_z_variance = 0.01F;

  waitForMeasurementUpdate(
    std::move(measurement), [](const EstimatorStatusFlags::UniquePtr & flags) {
      return flags->cs_ev_pos && flags->cs_ev_hgt;
    });
}

TEST_F(LocalPositionInterfaceTest, fuseEvVel) {
  auto measurement = std::make_unique<LocalPositionMeasurement>();
  measurement->velocity_xy = Eigen::Vector2f {0.f, 0.f};
  measurement->velocity_xy_variance = Eigen::Vector2f {0.1f, 0.1f};
  measurement->velocity_z = 0.0f;
  measurement->velocity_z_variance = 0.1f;

  waitForMeasurementUpdate(
    std::move(measurement), [](const EstimatorStatusFlags::UniquePtr & flags) {
      return flags->cs_ev_vel;
    });
}

TEST_F(LocalPositionInterfaceTest, fuseEvYaw) {
  auto measurement = std::make_unique<LocalPositionMeasurement>();
  measurement->attitude_quaternion =
    Eigen::Quaternionf(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ()));  // East
  measurement->attitude_variance = Eigen::Vector3f {0.1f, 0.1f, 0.1f};

  waitForMeasurementUpdate(
    std::move(measurement), [](const EstimatorStatusFlags::UniquePtr & flags) {
      return flags->cs_ev_yaw;
    });
}

TEST_F(LocalPositionInterfaceTest, fuseAll) {
  auto measurement = std::make_unique<LocalPositionMeasurement>();
  measurement->position_xy = Eigen::Vector2f {0.f, 0.f};
  measurement->position_xy_variance = Eigen::Vector2f {0.1f, 0.1f};
  measurement->position_z = 0.f;
  measurement->position_z_variance = 0.1f;
  measurement->velocity_xy = Eigen::Vector2f {0.f, 0.f};
  measurement->velocity_xy_variance = Eigen::Vector2f {0.1f, 0.1f};
  measurement->velocity_z = 0.0f;
  measurement->velocity_z_variance = 0.1f;
  measurement->attitude_quaternion =
    Eigen::Quaternionf(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ()));  // East
  measurement->attitude_variance = Eigen::Vector3f {0.1f, 0.1f, 0.1f};

  waitForMeasurementUpdate(
    std::move(measurement), [](const EstimatorStatusFlags::UniquePtr & flags) {
      return flags->cs_ev_pos && flags->cs_ev_hgt && flags->cs_ev_vel && flags->cs_ev_yaw;
    });
}
