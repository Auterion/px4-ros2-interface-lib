/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>

#include <atomic>
#include <future>
#include <px4_msgs/msg/message_format_request.hpp>
#include <px4_msgs/msg/message_format_response.hpp>
#include <px4_ros2/components/message_compatibility_check.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/utils/vehicle_command_sender.hpp>
#include <thread>

#include "fake_registration.hpp"

using namespace std::chrono_literals;

namespace {

// Declare after callback owners so spinning stops before those owners are destroyed.
class SpinningExecutor {
 public:
  SpinningExecutor(const std::shared_ptr<rclcpp::Node>& node, bool multithreaded)
  {
    if (multithreaded) {
      _executor =
          std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
    } else {
      _executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    }
    _executor->add_node(node);
    _ready_future = _ready.get_future();
    _timer = node->create_wall_timer(10ms, [this]() {
      if (!_notified.exchange(true)) {
        _ready.set_value();
      }
    });
    _thread = std::thread([this]() {
      try {
        _executor->spin();
      } catch (const std::exception& error) {
        ADD_FAILURE() << "Executor failed: " << error.what();
      }
    });
  }

  ~SpinningExecutor()
  {
    _executor->cancel();
    _thread.join();
  }

  bool ready() { return _ready_future.wait_for(3s) == std::future_status::ready; }

 private:
  std::unique_ptr<rclcpp::Executor> _executor;
  rclcpp::TimerBase::SharedPtr _timer;
  std::promise<void> _ready;
  std::future<void> _ready_future;
  std::atomic<bool> _notified{false};
  std::thread _thread;
};

class WaitSetMode : public px4_ros2::ModeBase {
 public:
  WaitSetMode(rclcpp::Node& node, const std::string& prefix, bool skip_setpoint_check = false)
      : ModeBase(node, Settings{"Wait Set Test"}, prefix),
        _rates(std::make_shared<px4_ros2::RatesSetpointType>(*this))
  {
    setSkipMessageCompatibilityCheck();
    overrideRegistration(std::make_shared<FakeRegistration>(node));
    if (skip_setpoint_check) {
      setSkipSetpointCheck();
    }
  }

 private:
  std::shared_ptr<px4_ros2::RatesSetpointType> _rates;
};

class DeferringExecutor : public px4_ros2::ModeExecutorBase {
 public:
  explicit DeferringExecutor(WaitSetMode& mode)
      : ModeExecutorBase(Settings{}.activate(Settings::Activation::ActivateAlways), mode)
  {
    setSkipMessageCompatibilityCheck();
    overrideRegistration(std::make_shared<FakeRegistration>(node()));
  }

  void onActivate() override
  {
    _activated = true;
    _activation.set_value();
  }
  void onDeactivate(DeactivateReason reason) override {}
  bool activated() const { return _activated; }
  std::future<void> activation() { return _activation.get_future(); }

 private:
  std::atomic<bool> _activated{false};
  std::promise<void> _activation;
};

}  // namespace

class WaitSetTest : public testing::TestWithParam<bool> {
 protected:
  void SetUp() override
  {
    static unsigned instance = 0;
    _prefix = "/wait_set_test_" + std::to_string(instance++) + "/";
    _node = std::make_shared<rclcpp::Node>("wait_set_test");
    _responder = std::make_shared<rclcpp::Node>("wait_set_responder");
  }

  template <typename Message>
  std::string topic(const std::string& name) const
  {
    return _prefix + name + px4_ros2::getMessageNameVersion<Message>();
  }

  std::string _prefix;
  std::shared_ptr<rclcpp::Node> _node;
  std::shared_ptr<rclcpp::Node> _responder;
};

TEST_P(WaitSetTest, WaitForFMUWhileExecutorSpins)
{
  auto status_pub = _responder->create_publisher<px4_msgs::msg::VehicleStatus>(
      topic<px4_msgs::msg::VehicleStatus>("fmu/out/vehicle_status"), rclcpp::QoS(1).best_effort());
  auto heartbeat = _responder->create_wall_timer(
      20ms, [status_pub]() { status_pub->publish(px4_msgs::msg::VehicleStatus{}); });
  SpinningExecutor responder_executor(_responder, false);
  SpinningExecutor executor(_node, GetParam());
  ASSERT_TRUE(responder_executor.ready());
  ASSERT_TRUE(executor.ready());

  for (int i = 0; i < 2; ++i) {
    bool received = false;
    ASSERT_NO_THROW(received = px4_ros2::waitForFMU(*_node, 3s, 1s, _prefix));
    EXPECT_TRUE(received);
  }
}

TEST_P(WaitSetTest, CommandWhileExecutorSpins)
{
  auto ack_pub = _responder->create_publisher<px4_msgs::msg::VehicleCommandAck>(
      topic<px4_msgs::msg::VehicleCommandAck>("fmu/out/vehicle_command_ack"),
      rclcpp::QoS(1).best_effort());
  auto command_sub = _responder->create_subscription<px4_msgs::msg::VehicleCommand>(
      topic<px4_msgs::msg::VehicleCommand>("fmu/in/vehicle_command"), rclcpp::QoS(1),
      [ack_pub](px4_msgs::msg::VehicleCommand::UniquePtr command) {
        px4_msgs::msg::VehicleCommandAck ack{};
        ack.command = command->command;
        ack.target_component = command->source_component;
        ack.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
        ack_pub->publish(ack);
      });
  px4_ros2::VehicleCommandSender sender(*_node, _prefix);
  SpinningExecutor responder_executor(_responder, false);
  SpinningExecutor executor(_node, GetParam());
  ASSERT_TRUE(responder_executor.ready());
  ASSERT_TRUE(executor.ready());

  px4_msgs::msg::VehicleCommand command{};
  command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME;
  for (int i = 0; i < 2; ++i) {
    px4_ros2::Result result{};
    ASSERT_NO_THROW(result = sender.sendCommandSync(command));
    EXPECT_EQ(result, px4_ros2::Result::Success);
  }
}

TEST_P(WaitSetTest, MessageCompatibilityWhileExecutorSpins)
{
  std::atomic<unsigned> requests{0};
  auto reply_pub = _responder->create_publisher<px4_msgs::msg::MessageFormatResponse>(
      topic<px4_msgs::msg::MessageFormatResponse>("fmu/out/message_format_response"),
      rclcpp::QoS(1).best_effort());
  auto request_sub = _responder->create_subscription<px4_msgs::msg::MessageFormatRequest>(
      topic<px4_msgs::msg::MessageFormatRequest>("fmu/in/message_format_request"), rclcpp::QoS(1),
      [reply_pub, &requests](px4_msgs::msg::MessageFormatRequest::UniquePtr request) {
        px4_msgs::msg::MessageFormatResponse reply{};
        reply.protocol_version = request->protocol_version;
        reply.topic_name = request->topic_name;
        reply.success = false;
        ++requests;
        reply_pub->publish(reply);
      });
  SpinningExecutor responder_executor(_responder, false);
  SpinningExecutor executor(_node, GetParam());
  ASSERT_TRUE(responder_executor.ready());
  ASSERT_TRUE(executor.ready());

  for (unsigned i = 0; i < 2; ++i) {
    bool compatible = true;
    ASSERT_NO_THROW(compatible = px4_ros2::messageCompatibilityCheck(
                        *_node, {{"fmu/out/vehicle_status", "VehicleStatus"}}, _prefix));
    EXPECT_FALSE(compatible);
    // An explicit rejection needs one request; losing the reply would exhaust retries.
    EXPECT_EQ(requests.load(), i + 1);
  }
}

TEST_P(WaitSetTest, SetpointCompatibilityWhileExecutorSpins)
{
  auto reply_pub = _responder->create_publisher<px4_msgs::msg::SetpointConfigReply>(
      topic<px4_msgs::msg::SetpointConfigReply>("fmu/out/setpoint_config_reply"),
      rclcpp::QoS(1).best_effort());
  auto request_sub = _responder->create_subscription<px4_msgs::msg::SetpointConfig>(
      topic<px4_msgs::msg::SetpointConfig>("fmu/in/setpoint_config"), rclcpp::QoS(1),
      [reply_pub](px4_msgs::msg::SetpointConfig::UniquePtr request) {
        EXPECT_FALSE(request->should_apply);
        px4_msgs::msg::SetpointConfigReply reply{};
        reply.source_id = request->source_id;
        reply.type = request->type;
        reply.result = px4_msgs::msg::SetpointConfigReply::RESULT_SUCCESS;
        reply.mode_req_angular_velocity = true;
        reply_pub->publish(reply);
      });
  WaitSetMode mode(*_node, _prefix);
  SpinningExecutor responder_executor(_responder, false);
  SpinningExecutor executor(_node, GetParam());
  ASSERT_TRUE(responder_executor.ready());
  ASSERT_TRUE(executor.ready());

  bool registered = false;
  ASSERT_NO_THROW(registered = mode.doRegister());
  EXPECT_TRUE(registered);
  EXPECT_TRUE(mode.modeRequirements().angular_velocity);
}

TEST_P(WaitSetTest, DeferFailsafesWhileExecutorSpins)
{
  WaitSetMode mode(*_node, _prefix, true);
  DeferringExecutor mode_executor(mode);
  ASSERT_TRUE(mode_executor.doRegister());
  auto activation = mode_executor.activation();

  std::atomic<bool> defer_requested{false};
  auto overrides_sub = _responder->create_subscription<px4_msgs::msg::ConfigOverrides>(
      topic<px4_msgs::msg::ConfigOverrides>("fmu/in/config_overrides_request"), rclcpp::QoS(1),
      [&defer_requested](px4_msgs::msg::ConfigOverrides::UniquePtr request) {
        if (request->source_type == px4_msgs::msg::ConfigOverrides::SOURCE_TYPE_MODE_EXECUTOR) {
          defer_requested = request->defer_failsafes;
        }
      });
  auto status_pub = _responder->create_publisher<px4_msgs::msg::VehicleStatus>(
      topic<px4_msgs::msg::VehicleStatus>("fmu/out/vehicle_status"), rclcpp::QoS(1).best_effort());
  auto heartbeat = _responder->create_wall_timer(20ms, [&, status_pub]() {
    // After activation, keep normal status callbacks quiescent until the synchronous
    // waiter exists. Its initial state checks must precede the acknowledgement.
    if (mode_executor.activated() &&
        (!defer_requested || status_pub->get_subscription_count() < 2)) {
      return;
    }
    px4_msgs::msg::VehicleStatus status{};
    status.executor_in_charge = mode_executor.id();
    status.nav_state = mode.id();
    status.failsafe_defer_state = defer_requested
                                      ? px4_msgs::msg::VehicleStatus::FAILSAFE_DEFER_STATE_ENABLED
                                      : px4_msgs::msg::VehicleStatus::FAILSAFE_DEFER_STATE_DISABLED;
    status_pub->publish(status);
  });
  SpinningExecutor responder_executor(_responder, false);
  SpinningExecutor executor(_node, GetParam());
  ASSERT_TRUE(responder_executor.ready());
  ASSERT_TRUE(executor.ready());

  ASSERT_EQ(activation.wait_for(3s), std::future_status::ready);
  // The default callback group must finish the activation callback before the call below.
  auto settled = std::make_shared<std::promise<void>>();
  auto settled_future = settled->get_future();
  auto notified = std::make_shared<std::atomic<bool>>(false);
  auto barrier = _node->create_wall_timer(10ms, [settled, notified]() {
    if (!notified->exchange(true)) {
      settled->set_value();
    }
  });
  ASSERT_EQ(settled_future.wait_for(3s), std::future_status::ready);
  bool deferred = false;
  ASSERT_NO_THROW(deferred = mode_executor.deferFailsafesSync(true));
  EXPECT_TRUE(deferred);
  EXPECT_TRUE(defer_requested);
}

INSTANTIATE_TEST_SUITE_P(Executors, WaitSetTest, testing::Bool(),
                         [](const testing::TestParamInfo<bool>& info) {
                           return info.param ? "MultiThreaded" : "SingleThreaded";
                         });
