/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/shared_subscription.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

using namespace std::chrono_literals;

class SharedSubscriptionTest : public testing::Test
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


TEST_F(SharedSubscriptionTest, State) {
  // Test that registered callbacks are called in order, and removed when the instance is deleted
  const std::string topic_name = "/unit_test/fmu/out/vehicle_status";
  auto & vehicle_status = SharedSubscription<px4_msgs::msg::VehicleStatus>::instance(
    *_node,
    topic_name);
  auto vehicle_status_pub = _node->create_publisher<px4_msgs::msg::VehicleStatus>(
    topic_name, rclcpp::QoS(
      1).best_effort());

  px4_msgs::msg::VehicleStatus pub_msg;
  std::array<std::optional<SharedSubscriptionCallbackInstance>, 3> callbacks;
  std::array<unsigned, 3> message_counters{};

  // Single registration
  unsigned counter = 1;
  bool got_message = false;
  callbacks[0] = vehicle_status.registerOnUpdateCallback(
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
  callbacks[1] = vehicle_status.registerOnUpdateCallback(
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
  callbacks[2] = vehicle_status.registerOnUpdateCallback(
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
  callbacks[1].reset();
  pub_msg.timestamp++;
  vehicle_status_pub->publish(pub_msg);
  ASSERT_TRUE(waitForUpdate(got_message));
  EXPECT_EQ(message_counters[0], 7U);
  EXPECT_EQ(message_counters[1], 5U);
  EXPECT_EQ(message_counters[2], 8U);

  // Add second again
  callbacks[1] = vehicle_status.registerOnUpdateCallback(
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

TEST_F(SharedSubscriptionTest, MultiInstance) {
  const std::string topic_name = "/unit_test/fmu/out/vehicle_status";
  auto & vehicle_status1 = SharedSubscription<px4_msgs::msg::VehicleStatus>::instance(
    *_node,
    topic_name);
  auto vehicle_status_pub1 = _node->create_publisher<px4_msgs::msg::VehicleStatus>(
    topic_name, rclcpp::QoS(
      1).best_effort());

  const std::string topic_prefix = "test";
  auto & vehicle_status2 = SharedSubscription<px4_msgs::msg::VehicleStatus>::instance(
    *_node,
    topic_prefix +
    topic_name);
  auto vehicle_status_pub2 = _node->create_publisher<px4_msgs::msg::VehicleStatus>(
    topic_prefix + topic_name, rclcpp::QoS(
      1).best_effort());

  ASSERT_NE(&vehicle_status1, &vehicle_status2);

  px4_msgs::msg::VehicleStatus pub_msg;
  std::array<std::optional<SharedSubscriptionCallbackInstance>, 2> callbacks;
  std::array<unsigned, 2> message_counters{};

  unsigned counter = 1;
  bool got_message = false;
  callbacks[0] = vehicle_status1.registerOnUpdateCallback(
    [&](const px4_msgs::msg::VehicleStatus::UniquePtr & msg)
    {
      EXPECT_EQ(msg->timestamp, pub_msg.timestamp);
      message_counters[0] = counter++;
      got_message = true;
    });
  callbacks[1] = vehicle_status2.registerOnUpdateCallback(
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

TEST_F(SharedSubscriptionTest, MultiTypes) {
  const std::string topic_name_status = "/unit_test/fmu/out/vehicle_status";
  auto & vehicle_status = SharedSubscription<px4_msgs::msg::VehicleStatus>::instance(
    *_node,
    topic_name_status);
  auto vehicle_status_pub = _node->create_publisher<px4_msgs::msg::VehicleStatus>(
    topic_name_status, rclcpp::QoS(
      1).best_effort());

  const std::string topic_name_attitude = "/unit_test/fmu/out/vehicle_attitude";
  auto & vehicle_attitude = SharedSubscription<px4_msgs::msg::VehicleAttitude>::instance(
    *_node,
    topic_name_attitude);
  auto vehicle_attitude_pub = _node->create_publisher<px4_msgs::msg::VehicleAttitude>(
    topic_name_attitude, rclcpp::QoS(
      1).best_effort());

  px4_msgs::msg::VehicleStatus pub_msg_status;
  px4_msgs::msg::VehicleAttitude pub_msg_attitude;
  std::array<std::optional<SharedSubscriptionCallbackInstance>, 2> callbacks;
  std::array<unsigned, 2> message_counters{};

  unsigned counter = 1;
  bool got_message = false;
  callbacks[0] = vehicle_status.registerOnUpdateCallback(
    [&](const px4_msgs::msg::VehicleStatus::UniquePtr & msg)
    {
      EXPECT_EQ(msg->timestamp, pub_msg_status.timestamp);
      message_counters[0] = counter++;
      got_message = true;
    });
  callbacks[1] = vehicle_attitude.registerOnUpdateCallback(
    [&](const px4_msgs::msg::VehicleAttitude::UniquePtr & msg)
    {
      EXPECT_EQ(msg->timestamp, pub_msg_attitude.timestamp);
      message_counters[1] = counter++;
      got_message = true;
    });

  pub_msg_status.timestamp = 1;
  vehicle_status_pub->publish(pub_msg_status);
  ASSERT_TRUE(waitForUpdate(got_message));
  EXPECT_EQ(message_counters[0], 1U);
  EXPECT_EQ(message_counters[1], 0U);

  pub_msg_attitude.timestamp = pub_msg_status.timestamp + 1;
  vehicle_attitude_pub->publish(pub_msg_attitude);
  ASSERT_TRUE(waitForUpdate(got_message));
  EXPECT_EQ(message_counters[0], 1U);
  EXPECT_EQ(message_counters[1], 2U);

  pub_msg_status.timestamp = pub_msg_attitude.timestamp + 1;
  vehicle_status_pub->publish(pub_msg_status);
  ASSERT_TRUE(waitForUpdate(got_message));
  EXPECT_EQ(message_counters[0], 3U);
  EXPECT_EQ(message_counters[1], 2U);
}
