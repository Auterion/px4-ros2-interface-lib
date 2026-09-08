/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "src/components/registration.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <future>
#include <px4_ros2/utils/message_version.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace std::chrono_literals;

class RegistrationTest : public testing::TestWithParam<bool> {
 protected:
  void SetUp() override
  {
    _node = std::make_shared<rclcpp::Node>("test_registration");
    if (GetParam()) {
      _executor =
          std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
    } else {
      _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }
  }

  void TearDown() override
  {
    _executor->cancel();
    if (_spin_thread.joinable()) {
      _spin_thread.join();
    }
  }

  std::shared_ptr<rclcpp::Node> _node;
  std::shared_ptr<rclcpp::Executor> _executor;
  std::thread _spin_thread;
};

TEST_P(RegistrationTest, RegisterWhileExecutorSpins)
{
  const std::string topic_prefix = "/test_registration/";
  Registration registration(*_node, topic_prefix);
  auto responder_node = std::make_shared<rclcpp::Node>("registration_responder");
  auto reply_pub = responder_node->create_publisher<px4_msgs::msg::RegisterExtComponentReply>(
      topic_prefix + "fmu/out/register_ext_component_reply" +
          px4_ros2::getMessageNameVersion<px4_msgs::msg::RegisterExtComponentReply>(),
      rclcpp::QoS(1).best_effort());
  auto request_sub =
      responder_node->create_subscription<px4_msgs::msg::RegisterExtComponentRequest>(
          topic_prefix + "fmu/in/register_ext_component_request" +
              px4_ros2::getMessageNameVersion<px4_msgs::msg::RegisterExtComponentRequest>(),
          rclcpp::QoS(1),
          [reply_pub](px4_msgs::msg::RegisterExtComponentRequest::UniquePtr request) {
            px4_msgs::msg::RegisterExtComponentReply reply{};
            reply.request_id = request->request_id;
            reply.name = request->name;
            reply.px4_ros2_api_version = request->px4_ros2_api_version;
            reply.success = true;
            reply.arming_check_id = 1;
            reply.mode_id = 100;
            reply.mode_executor_id = 2;
            reply_pub->publish(reply);
          });

  // A callback confirms that the executor has collected the node's subscriptions
  // into its wait set before registration attempts to use its own wait set.
  auto spinning = std::make_shared<std::promise<void>>();
  auto spinning_future = spinning->get_future();
  auto notified = std::make_shared<std::atomic<bool>>(false);
  auto timer = _node->create_wall_timer(10ms, [spinning, notified]() {
    if (!notified->exchange(true)) {
      spinning->set_value();
    }
  });
  _executor->add_node(_node);
  _executor->add_node(responder_node);
  _spin_thread = std::thread([this]() { _executor->spin(); });
  ASSERT_EQ(spinning_future.wait_for(3s), std::future_status::ready);

  RegistrationSettings settings{};
  settings.name = "Test Registration";
  settings.register_arming_check = true;
  settings.register_mode = true;
  settings.register_mode_executor = true;

  // Re-registering also verifies that the manual wait set releases the subscription.
  for (int i = 0; i < 2; ++i) {
    bool registered = false;
    ASSERT_NO_THROW(registered = registration.doRegister(settings));
    ASSERT_TRUE(registered);
    EXPECT_TRUE(registration.registered());
    EXPECT_EQ(registration.armingCheckId(), 1);
    EXPECT_EQ(registration.modeId(), 100);
    EXPECT_EQ(registration.modeExecutorId(), 2);
    EXPECT_EQ(registration.name(), settings.name);
    registration.doUnregister();
    EXPECT_FALSE(registration.registered());
  }
}

INSTANTIATE_TEST_SUITE_P(Executors, RegistrationTest, testing::Bool(),
                         [](const testing::TestParamInfo<bool>& info) {
                           return info.param ? "MultiThreaded" : "SingleThreaded";
                         });
