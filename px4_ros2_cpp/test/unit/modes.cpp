/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>

#include <functional>
#include <px4_msgs/msg/setpoint_config.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "fake_autopilot.hpp"
#include "fake_registration.hpp"
#include "utils/wait_for.hpp"

class TestMode : public px4_ros2::ModeBase {
 public:
  explicit TestMode(rclcpp::Node& node) : ModeBase(node, std::string("test"))
  {
    EXPECT_FALSE(modeRequirements().manual_control);
    EXPECT_FALSE(modeRequirements().angular_velocity);
    EXPECT_FALSE(modeRequirements().global_position);
    _manual_control_input = std::make_shared<px4_ros2::ManualControlInput>(*this);
    _rates_setpoint = std::make_shared<px4_ros2::RatesSetpointType>(*this);
    _global_position = std::make_shared<px4_ros2::OdometryGlobalPosition>(*this);
    EXPECT_TRUE(modeRequirements().manual_control);
    EXPECT_FALSE(modeRequirements()
                     .angular_velocity);  // Setpoint requirements are only set after registration
    EXPECT_TRUE(modeRequirements().global_position);

    setSkipMessageCompatibilityCheck();
    overrideRegistration(std::make_shared<FakeRegistration>(node));
    setSkipSetpointCheck();
  }

 private:
  std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input;
  std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;
  std::shared_ptr<px4_ros2::OdometryGlobalPosition> _global_position;
};

TEST(modes, modeRequirements)
{
  rclcpp::Node node("test_node");
  auto mode = std::make_shared<TestMode>(node);
  EXPECT_TRUE(mode->doRegister());
  EXPECT_TRUE(mode->modeRequirements().manual_control);
  EXPECT_TRUE(mode->modeRequirements().global_position);
  // Requirements from the setpoint are not set as we do not simulate the setpoint reply
  // (setSkipSetpointCheck())

  mode->modeRequirements().clearAll();
  EXPECT_FALSE(mode->modeRequirements().manual_control);
  EXPECT_FALSE(mode->modeRequirements().global_position);
}

TEST(modes, nodeWithMode)
{
  auto node_with_mode = std::make_shared<px4_ros2::NodeWithMode<TestMode>>("test_node");
  EXPECT_TRUE(node_with_mode->getMode().modeRequirements().manual_control);

  node_with_mode->getMode().modeRequirements().clearAll();
  EXPECT_FALSE(node_with_mode->getMode().modeRequirements().manual_control);
}

namespace {

/**
 * Mode with two setpoint types. It sends rates setpoints, or attitude setpoints after
 * useAttitude(true).
 */
class SetpointTestMode : public px4_ros2::ModeBase {
 public:
  explicit SetpointTestMode(rclcpp::Node& node) : ModeBase(node, Settings{"test"})
  {
    _rates_setpoint = std::make_shared<px4_ros2::RatesSetpointType>(*this);
    _attitude_setpoint = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);

    setSkipMessageCompatibilityCheck();
    overrideRegistration(std::make_shared<FakeRegistration>(node));
    setSkipSetpointCheck();
  }

  void updateSetpoint(float dt_s) override
  {
    ++_num_updates;
    if (!_send_setpoints) {
      return;
    }
    if (_use_attitude) {
      _attitude_setpoint->update(Eigen::Quaternionf::Identity(), Eigen::Vector3f{0.f, 0.f, -0.5f});
    } else {
      _rates_setpoint->update(Eigen::Vector3f::Zero(), Eigen::Vector3f{0.f, 0.f, -0.5f});
    }
  }

  px4_ros2::RatesSetpointType& rates() { return *_rates_setpoint; }
  px4_ros2::AttitudeSetpointType& attitude() { return *_attitude_setpoint; }
  void setSendSetpoints(bool send_setpoints) { _send_setpoints = send_setpoints; }
  void useAttitude(bool use_attitude) { _use_attitude = use_attitude; }
  int numUpdates() const { return _num_updates; }

 private:
  std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;
  std::shared_ptr<px4_ros2::AttitudeSetpointType> _attitude_setpoint;
  bool _send_setpoints{true};
  bool _use_attitude{false};
  int _num_updates{0};
};

constexpr auto kTypeRates = px4_msgs::msg::SetpointConfig::TYPE_RATES;
constexpr auto kTypeAttitude = px4_msgs::msg::SetpointConfig::TYPE_ATTITUDE;

/**
 * Wait for a number of setpoint updates of the mode, sent or not
 */
bool waitForUpdates(const std::shared_ptr<rclcpp::Node>& node, const SetpointTestMode& mode,
                    int num_updates)
{
  const int end = mode.numUpdates() + num_updates;
  return waitFor(node, [&mode, end] { return mode.numUpdates() >= end; });
}

/**
 * Wait until the FMU received a setpoint configuration, then for the reply to take effect
 */
bool waitForConfig(const std::shared_ptr<rclcpp::Node>& node, const SetpointConfigResponder& fmu,
                   const SetpointTestMode& mode)
{
  return waitFor(node, [&fmu] { return !fmu.configs.empty(); }) && waitForUpdates(node, mode, 10);
}

/**
 * Check that the setpoint configurations the FMU received are all of one setpoint type and
 * timeout, and clear them
 */
void expectConfigs(SetpointConfigResponder& fmu, uint16_t type, uint16_t timeout_ms)
{
  EXPECT_FALSE(fmu.configs.empty());
  for (const auto& setpoint_config : fmu.configs) {
    EXPECT_EQ(setpoint_config.type, type);
    EXPECT_EQ(setpoint_config.timeout_ms, timeout_ms);
  }
  fmu.configs.clear();
}

px4_msgs::msg::SetpointConfig setpointConfigOnActivation(
    const std::function<void(SetpointTestMode&)>& setup)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  FakeAutopilot fake_autopilot(node, "");
  const SetpointConfigResponder& fmu = fake_autopilot.setpointConfigResponder();
  auto mode = std::make_shared<SetpointTestMode>(*node);
  setup(*mode);
  EXPECT_TRUE(mode->doRegister());

  fake_autopilot.setModeAndArm(mode->id());
  EXPECT_TRUE(waitFor(node, [&fmu] { return !fmu.configs.empty(); }));
  return fmu.configs.empty() ? px4_msgs::msg::SetpointConfig{} : fmu.configs.front();
}

}  // namespace

TEST(modes, setpointConfigReplyAfterDeactivation)
{
  // A reply to a setpoint configuration that only arrives after the mode got deactivated must not
  // keep the next activation from sending the configuration again
  auto node = std::make_shared<rclcpp::Node>("test_node");
  FakeAutopilot fake_autopilot(node, "");
  SetpointConfigResponder& fmu = fake_autopilot.setpointConfigResponder();
  auto mode = std::make_shared<SetpointTestMode>(*node);
  ASSERT_TRUE(mode->doRegister());

  fmu.reply = false;
  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitFor(node, [&fmu] { return !fmu.configs.empty(); }));
  fake_autopilot.setModeAndArm(0);
  ASSERT_TRUE(waitFor(node, [&mode] { return !mode->isActive(); }));
  fmu.sendReply(mode->id(), kTypeRates);
  const auto reply_sent = std::chrono::steady_clock::now();
  waitFor(node, [reply_sent] { return std::chrono::steady_clock::now() > reply_sent + 100ms; });

  fmu.configs.clear();
  fmu.reply = true;
  fake_autopilot.setModeAndArm(mode->id());
  EXPECT_TRUE(waitFor(node, [&fmu] { return !fmu.configs.empty(); }));
}

TEST(modes, setpointConfigWithoutTimeout)
{
  // Without a setpoint timeout the setpoint configurations are the same as before timeouts existed:
  // sent for a setpoint type activation and with every setpoint update until the FMU replied,
  // always with timeout 0, and none from completed()
  auto node = std::make_shared<rclcpp::Node>("test_node");
  FakeAutopilot fake_autopilot(node, "");
  SetpointConfigResponder& fmu = fake_autopilot.setpointConfigResponder();
  auto mode = std::make_shared<SetpointTestMode>(*node);
  ASSERT_TRUE(mode->doRegister());

  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeRates, 0);

  // completed() with and without setpoints after it
  mode->completed(px4_ros2::Result::Success);
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  mode->setSendSetpoints(false);
  mode->completed(px4_ros2::Result::Success);
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  EXPECT_TRUE(fmu.configs.empty());

  // Another setpoint type
  mode->setSendSetpoints(true);
  mode->useAttitude(true);
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeAttitude, 0);
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  EXPECT_TRUE(fmu.configs.empty());

  // Activated again without a reply from the FMU: resent with every setpoint update, and no more
  // once the mode completed and sends no setpoints
  fake_autopilot.setModeAndArm(0);
  ASSERT_TRUE(waitFor(node, [&mode] { return !mode->isActive(); }));
  fmu.reply = false;
  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitFor(node, [&fmu] { return fmu.configs.size() >= 3; }));
  mode->setSendSetpoints(false);
  mode->completed(px4_ros2::Result::Success);
  ASSERT_TRUE(waitForUpdates(node, *mode, 5));  // Configurations still on their way
  const auto num_configs = fmu.configs.size();
  ASSERT_TRUE(waitForUpdates(node, *mode, 20));
  EXPECT_EQ(fmu.configs.size(), num_configs);
  expectConfigs(fmu, kTypeAttitude, 0);
}

TEST(modes, setpointTimeoutDefault)
{
  const auto setpoint_config = setpointConfigOnActivation([](SetpointTestMode&) {});
  EXPECT_EQ(setpoint_config.type, kTypeRates);
  EXPECT_EQ(setpoint_config.timeout_ms, 0);
}

TEST(modes, setpointTimeout)
{
  const auto setpoint_config = setpointConfigOnActivation(
      [](SetpointTestMode& mode) { mode.rates().setTimeout(std::chrono::milliseconds(150)); });
  EXPECT_EQ(setpoint_config.type, kTypeRates);
  EXPECT_EQ(setpoint_config.timeout_ms, 150);
}

TEST(modes, setpointTimeoutLimited)
{
  EXPECT_EQ(setpointConfigOnActivation([](SetpointTestMode& mode) {
              mode.rates().setTimeout(std::chrono::minutes(2));
            }).timeout_ms,
            65535);
  EXPECT_EQ(setpointConfigOnActivation([](SetpointTestMode& mode) {
              mode.rates().setTimeout(std::chrono::milliseconds(-5));
            }).timeout_ms,
            0);
}

TEST(modes, setpointTimeoutPerSetpointType)
{
  // Each setpoint type sends its own timeout when it gets activated
  auto node = std::make_shared<rclcpp::Node>("test_node");
  FakeAutopilot fake_autopilot(node, "");
  SetpointConfigResponder& fmu = fake_autopilot.setpointConfigResponder();
  auto mode = std::make_shared<SetpointTestMode>(*node);
  mode->rates().setTimeout(std::chrono::milliseconds(100));
  mode->attitude().setTimeout(std::chrono::milliseconds(300));
  ASSERT_TRUE(mode->doRegister());

  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeRates, 100);
  mode->useAttitude(true);
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeAttitude, 300);
  mode->useAttitude(false);
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeRates, 100);
}

TEST(modes, setpointTimeoutChanged)
{
  // A changed timeout of the setpoint type in use is sent with the next setpoint update, once
  auto node = std::make_shared<rclcpp::Node>("test_node");
  FakeAutopilot fake_autopilot(node, "");
  SetpointConfigResponder& fmu = fake_autopilot.setpointConfigResponder();
  auto mode = std::make_shared<SetpointTestMode>(*node);
  mode->rates().setTimeout(std::chrono::milliseconds(100));
  ASSERT_TRUE(mode->doRegister());

  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeRates, 100);

  // The same value
  mode->rates().setTimeout(std::chrono::milliseconds(100));
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  EXPECT_TRUE(fmu.configs.empty());

  // A new value, and nothing more once the FMU replied
  mode->rates().setTimeout(std::chrono::milliseconds(250));
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeRates, 250);
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  EXPECT_TRUE(fmu.configs.empty());

  // A setpoint type that is not in use: sent when it gets activated
  mode->attitude().setTimeout(std::chrono::milliseconds(300));
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  EXPECT_TRUE(fmu.configs.empty());
  mode->useAttitude(true);
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeAttitude, 300);
}

TEST(modes, setpointTimeoutCompleted)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  FakeAutopilot fake_autopilot(node, "");
  SetpointConfigResponder& fmu = fake_autopilot.setpointConfigResponder();
  auto mode = std::make_shared<SetpointTestMode>(*node);
  mode->rates().setTimeout(std::chrono::milliseconds(150));
  ASSERT_TRUE(mode->doRegister());

  // Activated: the timeout, sent until the FMU replied
  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeRates, 150);
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  EXPECT_TRUE(fmu.configs.empty());

  // Completed, and no setpoints anymore: the same setpoint type with timeout 0, resent until the
  // FMU replied
  fmu.reply = false;
  mode->setSendSetpoints(false);
  mode->completed(px4_ros2::Result::Success);
  ASSERT_TRUE(waitFor(node, [&fmu] { return fmu.configs.size() >= 3; }));
  for (const auto& setpoint_config : fmu.configs) {
    EXPECT_EQ(setpoint_config.source_id, mode->id());
  }
  expectConfigs(fmu, kTypeRates, 0);
  fmu.reply = true;
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  fmu.configs.clear();
  mode->completed(px4_ros2::Result::Success);  // modes may call it repeatedly
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  EXPECT_TRUE(fmu.configs.empty());

  // Activated again: the timeout again
  fake_autopilot.setModeAndArm(0);
  ASSERT_TRUE(waitFor(node, [&mode] { return !mode->isActive(); }));
  mode->setSendSetpoints(true);
  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeRates, 150);
}

TEST(modes, setpointTimeoutAfterCompleted)
{
  // After completed() every setpoint configuration of the activation has timeout 0, also for a
  // changed timeout and for another setpoint type. The next activation sends the timeouts again.
  auto node = std::make_shared<rclcpp::Node>("test_node");
  FakeAutopilot fake_autopilot(node, "");
  SetpointConfigResponder& fmu = fake_autopilot.setpointConfigResponder();
  auto mode = std::make_shared<SetpointTestMode>(*node);
  mode->rates().setTimeout(std::chrono::milliseconds(100));
  mode->attitude().setTimeout(std::chrono::milliseconds(300));
  ASSERT_TRUE(mode->doRegister());

  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeRates, 100);

  mode->completed(px4_ros2::Result::Success);  // keeps sending setpoints
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeRates, 0);
  mode->rates().setTimeout(std::chrono::milliseconds(250));
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeRates, 0);
  mode->useAttitude(true);
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeAttitude, 0);

  fake_autopilot.setModeAndArm(0);
  ASSERT_TRUE(waitFor(node, [&mode] { return !mode->isActive(); }));
  fmu.configs.clear();
  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeAttitude, 300);
  mode->useAttitude(false);
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeRates, 250);
}

TEST(modes, setpointTimeoutOtherTypeAfterCompleted)
{
  // completed() while the setpoint type in use has no timeout: another setpoint type with a
  // timeout still sends 0 for the rest of the activation
  auto node = std::make_shared<rclcpp::Node>("test_node");
  FakeAutopilot fake_autopilot(node, "");
  SetpointConfigResponder& fmu = fake_autopilot.setpointConfigResponder();
  auto mode = std::make_shared<SetpointTestMode>(*node);
  mode->attitude().setTimeout(std::chrono::milliseconds(300));
  ASSERT_TRUE(mode->doRegister());

  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeRates, 0);

  mode->completed(px4_ros2::Result::Success);  // keeps sending setpoints
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  EXPECT_TRUE(fmu.configs.empty());
  mode->useAttitude(true);
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeAttitude, 0);

  fake_autopilot.setModeAndArm(0);
  ASSERT_TRUE(waitFor(node, [&mode] { return !mode->isActive(); }));
  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeAttitude, 300);
}

TEST(modes, setpointTimeoutSetAfterCompleted)
{
  // A timeout set after completed() in a mode that had none applies from the next activation on
  auto node = std::make_shared<rclcpp::Node>("test_node");
  FakeAutopilot fake_autopilot(node, "");
  SetpointConfigResponder& fmu = fake_autopilot.setpointConfigResponder();
  auto mode = std::make_shared<SetpointTestMode>(*node);
  ASSERT_TRUE(mode->doRegister());

  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeRates, 0);

  mode->completed(px4_ros2::Result::Success);  // keeps sending setpoints
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  EXPECT_TRUE(fmu.configs.empty());
  mode->rates().setTimeout(std::chrono::milliseconds(100));
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeRates, 0);
  mode->attitude().setTimeout(std::chrono::milliseconds(300));
  mode->useAttitude(true);
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeAttitude, 0);

  fake_autopilot.setModeAndArm(0);
  ASSERT_TRUE(waitFor(node, [&mode] { return !mode->isActive(); }));
  mode->useAttitude(false);
  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitForConfig(node, fmu, *mode));
  expectConfigs(fmu, kTypeRates, 100);
}

TEST(modes, setpointTimeoutWarning)
{
  // A timeout shorter than the setpoint update interval is reported once per activation. The node
  // has its own name: other tests set the log level of "test_node" to FATAL.
  auto node = std::make_shared<rclcpp::Node>("setpoint_timeout_warning_test_node");
  std::vector<std::string> warnings;
  const auto log_sub = node->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", rclcpp::QoS(100), [&warnings](const rcl_interfaces::msg::Log& msg) {
        if (msg.level == rcl_interfaces::msg::Log::WARN &&
            msg.name == "setpoint_timeout_warning_test_node") {
          warnings.push_back(msg.msg);
        }
      });
  FakeAutopilot fake_autopilot(node, "");
  SetpointConfigResponder& fmu = fake_autopilot.setpointConfigResponder();
  fmu.reply = false;  // The configuration is resent with every setpoint update
  auto mode = std::make_shared<SetpointTestMode>(*node);
  mode->setSetpointUpdateRate(50.f);
  mode->rates().setTimeout(std::chrono::milliseconds(10));
  mode->attitude().setTimeout(std::chrono::milliseconds(30));
  ASSERT_TRUE(mode->doRegister());

  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitFor(node, [&fmu] { return fmu.configs.size() >= 5; }));
  ASSERT_TRUE(waitFor(node, [&warnings] { return !warnings.empty(); }));
  ASSERT_TRUE(waitForUpdates(node, *mode, 5));
  ASSERT_EQ(warnings.size(), 1U);
  EXPECT_NE(warnings.front().find("10 ms"), std::string::npos) << warnings.front();
  EXPECT_NE(warnings.front().find("20 ms"), std::string::npos) << warnings.front();

  // Activated again: reported again
  fake_autopilot.setModeAndArm(0);
  ASSERT_TRUE(waitFor(node, [&mode] { return !mode->isActive(); }));
  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitFor(node, [&warnings] { return warnings.size() >= 2; }));

  // Longer than the update interval, or 0: nothing
  fake_autopilot.setModeAndArm(0);
  ASSERT_TRUE(waitFor(node, [&mode] { return !mode->isActive(); }));
  mode->useAttitude(true);
  fmu.configs.clear();
  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitFor(node, [&fmu] { return fmu.configs.size() >= 5; }));
  mode->attitude().setTimeout(std::chrono::milliseconds(0));
  ASSERT_TRUE(waitForUpdates(node, *mode, 5));
  fmu.configs.clear();
  ASSERT_TRUE(waitFor(node, [&fmu] { return fmu.configs.size() >= 5; }));
  expectConfigs(fmu, kTypeAttitude, 0);
  EXPECT_EQ(warnings.size(), 2U);
}
