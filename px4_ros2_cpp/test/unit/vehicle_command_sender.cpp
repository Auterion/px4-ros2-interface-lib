/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>

#include <thread>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <px4_ros2/utils/vehicle_command_sender.hpp>
#include <px4_ros2/vehicle_state/home_position_setter.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace {

constexpr const char* kTopicPrefix = "/test_cmd_sender/";

/**
 * Simulates PX4's command-ACK behavior: subscribes to vehicle_command,
 * publishes a matching VehicleCommandAck on receipt.
 */
class FakeCommandResponder {
 public:
  FakeCommandResponder(rclcpp::Node& node, const std::string& topic_prefix, uint8_t ack_result)
  {
    _ack_pub = node.create_publisher<px4_msgs::msg::VehicleCommandAck>(
        topic_prefix + "fmu/out/vehicle_command_ack" +
            px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommandAck>(),
        rclcpp::QoS(1).best_effort());

    _cmd_sub = node.create_subscription<px4_msgs::msg::VehicleCommand>(
        topic_prefix + "fmu/in/vehicle_command" +
            px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommand>(),
        rclcpp::QoS(1),
        [this, ack_result](px4_msgs::msg::VehicleCommand::UniquePtr msg) {
          px4_msgs::msg::VehicleCommandAck ack{};
          ack.command = msg->command;
          ack.target_component = msg->source_component;
          ack.result = ack_result;
          _ack_pub->publish(ack);
        });
  }

 private:
  rclcpp::Publisher<px4_msgs::msg::VehicleCommandAck>::SharedPtr _ack_pub;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr _cmd_sub;
};

}  // namespace

/**
 * Test fixture: uses a separate responder node (with its own executor thread)
 * so that VehicleCommandSender's internal WaitSet on the test node has no
 * conflict with the executor driving the fake responder callbacks.
 */
class VehicleCommandSenderTest : public testing::Test {
 protected:
  void SetUp() override
  {
    _test_node = std::make_shared<rclcpp::Node>("test_cmd_node");
    _responder_node = std::make_shared<rclcpp::Node>("responder_node");
    _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _executor->add_node(_responder_node);
  }

  void TearDown() override
  {
    _executor->cancel();
    if (_spin_thread.joinable()) {
      _spin_thread.join();
    }
  }

  void startResponder(
      uint8_t ack_result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
  {
    _responder =
        std::make_unique<FakeCommandResponder>(*_responder_node, kTopicPrefix, ack_result);
    _spin_thread = std::thread([this]() { _executor->spin(); });
    std::this_thread::sleep_for(50ms);  // allow DDS discovery
  }

  std::shared_ptr<rclcpp::Node> _test_node;
  std::shared_ptr<rclcpp::Node> _responder_node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> _executor;
  std::unique_ptr<FakeCommandResponder> _responder;
  std::thread _spin_thread;
};

// ---------------------------------------------------------------------------
// VehicleCommandSender
// ---------------------------------------------------------------------------

TEST_F(VehicleCommandSenderTest, Accepted)
{
  startResponder(px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED);
  px4_ros2::VehicleCommandSender sender(*_test_node, kTopicPrefix);

  px4_msgs::msg::VehicleCommand cmd{};
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME;

  EXPECT_EQ(sender.sendCommandSync(cmd), px4_ros2::Result::Success);
}

TEST_F(VehicleCommandSenderTest, Rejected)
{
  startResponder(px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED);
  px4_ros2::VehicleCommandSender sender(*_test_node, kTopicPrefix);

  px4_msgs::msg::VehicleCommand cmd{};
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME;

  EXPECT_EQ(sender.sendCommandSync(cmd), px4_ros2::Result::Rejected);
}

TEST_F(VehicleCommandSenderTest, Timeout)
{
  // Create an ACK publisher so the "wait for publisher" loop exits quickly,
  // but do NOT set up a command responder — no ACK will ever be sent.
  _responder_node->create_publisher<px4_msgs::msg::VehicleCommandAck>(
      std::string(kTopicPrefix) + "fmu/out/vehicle_command_ack" +
          px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommandAck>(),
      rclcpp::QoS(1).best_effort());
  _spin_thread = std::thread([this]() { _executor->spin(); });
  std::this_thread::sleep_for(50ms);

  px4_ros2::VehicleCommandSender sender(*_test_node, kTopicPrefix);
  px4_msgs::msg::VehicleCommand cmd{};
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME;

  EXPECT_EQ(sender.sendCommandSync(cmd), px4_ros2::Result::Timeout);
}

// ---------------------------------------------------------------------------
// HomePositionSetter (uses VehicleCommandSender internally)
// ---------------------------------------------------------------------------

TEST_F(VehicleCommandSenderTest, HomeSetCurrentPosition)
{
  startResponder();
  px4_ros2::Context context(*_test_node, kTopicPrefix);
  px4_ros2::HomePositionSetter setter(context);

  EXPECT_EQ(setter.setHomeToCurrentPosition(), px4_ros2::Result::Success);
}

TEST_F(VehicleCommandSenderTest, HomeSetCoordinates)
{
  startResponder();
  px4_ros2::Context context(*_test_node, kTopicPrefix);
  px4_ros2::HomePositionSetter setter(context);

  EXPECT_EQ(setter.setHome(47.398, 8.546, 490.f), px4_ros2::Result::Success);
}

TEST_F(VehicleCommandSenderTest, HomeSetGpsGlobalOrigin)
{
  startResponder();
  px4_ros2::Context context(*_test_node, kTopicPrefix);
  px4_ros2::HomePositionSetter setter(context);

  EXPECT_EQ(setter.setGpsGlobalOrigin(47.397742, 8.545594, 488.f), px4_ros2::Result::Success);
}
