/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <src/components/shared_vehicle_status.hpp>
#include <px4_ros2/utils/message_version.hpp>

using namespace std::chrono_literals;

class VehicleStatusTest : public testing::Test
{
protected:
  void SetUp() override
  {
    _node = std::make_shared<rclcpp::Node>("test_node");
    _executor.add_node(_node);
  }

  bool waitForUpdate(bool & got_message)
  {
    got_message = false;
    auto start_time = _node->get_clock()->now();
    while (!got_message) {
      rclcpp::sleep_for(kSleepInterval);
      _executor.spin_some();
      const auto elapsed_time = _node->get_clock()->now() - start_time;
      if (elapsed_time >= kTimeoutDuration) {
        return got_message;
      }
    }
    return true;
  }

  std::shared_ptr<rclcpp::Node> _node;
  rclcpp::executors::SingleThreadedExecutor _executor;

  static constexpr std::chrono::seconds kTimeoutDuration{3s};
  static constexpr std::chrono::milliseconds kSleepInterval{10ms};
};


TEST_F(VehicleStatusTest, State) {
  // Test that registered callbacks are called in order, and removed when the token is deleted
  SharedVehicleStatus & vehicle_status = SharedVehicleStatus::instance(*_node);
  auto vehicle_status_pub = _node->create_publisher<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(), rclcpp::QoS(
      1).best_effort());

  px4_msgs::msg::VehicleStatus pub_msg;
  std::array<std::optional<SharedVehicleStatusToken>, 3> tokens;
  std::array<unsigned, 3> message_counters{};

  // Single registration
  unsigned counter = 1;
  bool got_message = false;
  tokens[0] = vehicle_status.registerVehicleStatusUpdatedCallback(
    [&](const px4_msgs::msg::VehicleStatus::UniquePtr & msg)
    {
      EXPECT_EQ(msg->timestamp, pub_msg.timestamp);
      message_counters[0] = counter++;
      got_message = true;
    });

  pub_msg.timestamp = 1;
  vehicle_status_pub->publish(pub_msg);
  ASSERT_TRUE(waitForUpdate(got_message));
  EXPECT_EQ(message_counters[0], 1U);
  EXPECT_EQ(message_counters[1], 0U);
  EXPECT_EQ(message_counters[2], 0U);

  // Add another one
  tokens[1] = vehicle_status.registerVehicleStatusUpdatedCallback(
    [&](const px4_msgs::msg::VehicleStatus::UniquePtr & msg)
    {
      EXPECT_EQ(msg->timestamp, pub_msg.timestamp);
      message_counters[1] = counter++;
    });

  pub_msg.timestamp++;
  vehicle_status_pub->publish(pub_msg);
  ASSERT_TRUE(waitForUpdate(got_message));
  EXPECT_EQ(message_counters[0], 2U);
  EXPECT_EQ(message_counters[1], 3U);
  EXPECT_EQ(message_counters[2], 0U);

  // Add another one
  tokens[2] = vehicle_status.registerVehicleStatusUpdatedCallback(
    [&](const px4_msgs::msg::VehicleStatus::UniquePtr & msg)
    {
      EXPECT_EQ(msg->timestamp, pub_msg.timestamp);
      message_counters[2] = counter++;
    });

  pub_msg.timestamp++;
  vehicle_status_pub->publish(pub_msg);
  ASSERT_TRUE(waitForUpdate(got_message));
  EXPECT_EQ(message_counters[0], 4U);
  EXPECT_EQ(message_counters[1], 5U);
  EXPECT_EQ(message_counters[2], 6U);

  // Remove the second
  tokens[1].reset();
  pub_msg.timestamp++;
  vehicle_status_pub->publish(pub_msg);
  ASSERT_TRUE(waitForUpdate(got_message));
  EXPECT_EQ(message_counters[0], 7U);
  EXPECT_EQ(message_counters[1], 5U);
  EXPECT_EQ(message_counters[2], 8U);

  // Add second again
  tokens[1] = vehicle_status.registerVehicleStatusUpdatedCallback(
    [&](const px4_msgs::msg::VehicleStatus::UniquePtr & msg)
    {
      EXPECT_EQ(msg->timestamp, pub_msg.timestamp);
      message_counters[1] = counter++;
    });

  pub_msg.timestamp++;
  vehicle_status_pub->publish(pub_msg);
  ASSERT_TRUE(waitForUpdate(got_message));
  EXPECT_EQ(message_counters[0], 9U);
  EXPECT_EQ(message_counters[1], 11U);
  EXPECT_EQ(message_counters[2], 10U);
}

TEST_F(VehicleStatusTest, MultiInstance) {
  SharedVehicleStatus & vehicle_status1 = SharedVehicleStatus::instance(*_node);
  auto vehicle_status_pub1 = _node->create_publisher<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(), rclcpp::QoS(
      1).best_effort());

  const std::string topic_prefix = "test/";
  SharedVehicleStatus & vehicle_status2 = SharedVehicleStatus::instance(*_node, topic_prefix);
  auto vehicle_status_pub2 = _node->create_publisher<px4_msgs::msg::VehicleStatus>(
    topic_prefix + "fmu/out/vehicle_status" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(), rclcpp::QoS(
      1).best_effort());

  ASSERT_NE(&vehicle_status1, &vehicle_status2);

  px4_msgs::msg::VehicleStatus pub_msg;
  std::array<std::optional<SharedVehicleStatusToken>, 2> tokens;
  std::array<unsigned, 2> message_counters{};

  unsigned counter = 1;
  bool got_message = false;
  tokens[0] = vehicle_status1.registerVehicleStatusUpdatedCallback(
    [&](const px4_msgs::msg::VehicleStatus::UniquePtr & msg)
    {
      EXPECT_EQ(msg->timestamp, pub_msg.timestamp);
      message_counters[0] = counter++;
      got_message = true;
    });
  tokens[1] = vehicle_status2.registerVehicleStatusUpdatedCallback(
    [&](const px4_msgs::msg::VehicleStatus::UniquePtr & msg)
    {
      EXPECT_EQ(msg->timestamp, pub_msg.timestamp);
      message_counters[1] = counter++;
      got_message = true;
    });

  pub_msg.timestamp = 1;
  vehicle_status_pub1->publish(pub_msg);
  ASSERT_TRUE(waitForUpdate(got_message));
  EXPECT_EQ(message_counters[0], 1U);
  EXPECT_EQ(message_counters[1], 0U);

  pub_msg.timestamp++;
  vehicle_status_pub2->publish(pub_msg);
  ASSERT_TRUE(waitForUpdate(got_message));
  EXPECT_EQ(message_counters[0], 1U);
  EXPECT_EQ(message_counters[1], 2U);
}
