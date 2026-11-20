/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>

#include <px4_msgs/msg/setpoint_config.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <rclcpp/rclcpp.hpp>

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

class SetpointTestMode : public px4_ros2::ModeBase {
 public:
  explicit SetpointTestMode(rclcpp::Node& node, const Settings& settings) : ModeBase(node, settings)
  {
    _rates_setpoint = std::make_shared<px4_ros2::RatesSetpointType>(*this);

    setSkipMessageCompatibilityCheck();
    overrideRegistration(std::make_shared<FakeRegistration>(node));
    setSkipSetpointCheck();
  }

  void updateSetpoint(float dt_s) override
  {
    ++_num_updates;
    if (_send_setpoints) {
      _rates_setpoint->update(Eigen::Vector3f::Zero(), Eigen::Vector3f{0.f, 0.f, -0.5f});
    }
  }

  void setSendSetpoints(bool send_setpoints) { _send_setpoints = send_setpoints; }
  int numUpdates() const { return _num_updates; }

 private:
  std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;
  bool _send_setpoints{true};
  int _num_updates{0};
};

TEST(modes, setpointConfigReplyAfterDeactivation)
{
  // A reply to a setpoint configuration that only arrives after the mode got deactivated must not
  // keep the next activation from sending the configuration again
  auto node = std::make_shared<rclcpp::Node>("test_node");
  FakeAutopilot fake_autopilot(node, "");
  SetpointConfigResponder& fmu = fake_autopilot.setpointConfigResponder();
  auto mode = std::make_shared<SetpointTestMode>(*node, px4_ros2::ModeBase::Settings{"test"});
  ASSERT_TRUE(mode->doRegister());

  fmu.reply = false;
  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitFor(node, [&fmu] { return !fmu.configs.empty(); }));
  fake_autopilot.setModeAndArm(0);
  ASSERT_TRUE(waitFor(node, [&mode] { return !mode->isActive(); }));
  fmu.sendReply(mode->id(), px4_msgs::msg::SetpointConfig::TYPE_RATES);
  const auto reply_sent = std::chrono::steady_clock::now();
  waitFor(node, [reply_sent] { return std::chrono::steady_clock::now() > reply_sent + 100ms; });

  fmu.configs.clear();
  fmu.reply = true;
  fake_autopilot.setModeAndArm(mode->id());
  EXPECT_TRUE(waitFor(node, [&fmu] { return !fmu.configs.empty(); }));
}

namespace {

px4_msgs::msg::SetpointConfig setpointConfigOnActivation(
    const px4_ros2::ModeBase::Settings& settings)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  FakeAutopilot fake_autopilot(node, "");
  const SetpointConfigResponder& fmu = fake_autopilot.setpointConfigResponder();
  auto mode = std::make_shared<SetpointTestMode>(*node, settings);
  EXPECT_TRUE(mode->doRegister());

  fake_autopilot.setModeAndArm(mode->id());
  EXPECT_TRUE(waitFor(node, [&fmu] { return !fmu.configs.empty(); }));
  return fmu.configs.empty() ? px4_msgs::msg::SetpointConfig{} : fmu.configs.front();
}

/**
 * Wait for a number of setpoint updates of the mode, sent or not
 */
bool waitForUpdates(const std::shared_ptr<rclcpp::Node>& node, const SetpointTestMode& mode,
                    int num_updates)
{
  const int end = mode.numUpdates() + num_updates;
  return waitFor(node, [&mode, end] { return mode.numUpdates() >= end; });
}

}  // namespace

TEST(modes, setpointTimeoutDefault)
{
  const auto setpoint_config = setpointConfigOnActivation(px4_ros2::ModeBase::Settings{"test"});
  EXPECT_EQ(setpoint_config.type, px4_msgs::msg::SetpointConfig::TYPE_RATES);
  EXPECT_EQ(setpoint_config.timeout_ms, 0);
}

TEST(modes, setpointTimeout)
{
  const auto setpoint_config = setpointConfigOnActivation(
      px4_ros2::ModeBase::Settings{"test"}.setpointTimeout(std::chrono::milliseconds(150)));
  EXPECT_EQ(setpoint_config.type, px4_msgs::msg::SetpointConfig::TYPE_RATES);
  EXPECT_EQ(setpoint_config.timeout_ms, 150);
}

TEST(modes, setpointTimeoutLimited)
{
  const auto setpoint_config = setpointConfigOnActivation(
      px4_ros2::ModeBase::Settings{"test"}.setpointTimeout(std::chrono::minutes(2)));
  EXPECT_EQ(setpoint_config.timeout_ms, 65535);
}

TEST(modes, setpointTimeoutCompleted)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  FakeAutopilot fake_autopilot(node, "");
  SetpointConfigResponder& fmu = fake_autopilot.setpointConfigResponder();
  auto mode = std::make_shared<SetpointTestMode>(
      *node, px4_ros2::ModeBase::Settings{"test"}.setpointTimeout(std::chrono::milliseconds(150)));
  ASSERT_TRUE(mode->doRegister());

  // Activated: the configured timeout, sent until the FMU replied
  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitFor(node, [&fmu] { return !fmu.configs.empty(); }));
  EXPECT_EQ(fmu.configs.front().timeout_ms, 150);
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  fmu.configs.clear();
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  EXPECT_TRUE(fmu.configs.empty());

  // Completed, and no setpoints anymore: the same setpoint type with timeout 0, resent until the
  // FMU replied
  fmu.reply = false;
  mode->setSendSetpoints(false);
  mode->completed(px4_ros2::Result::Success);
  ASSERT_TRUE(waitFor(node, [&fmu] { return fmu.configs.size() >= 3; }));
  for (const auto& setpoint_config : fmu.configs) {
    EXPECT_EQ(setpoint_config.type, px4_msgs::msg::SetpointConfig::TYPE_RATES);
    EXPECT_EQ(setpoint_config.source_id, mode->id());
    EXPECT_EQ(setpoint_config.timeout_ms, 0);
  }
  fmu.reply = true;
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  fmu.configs.clear();
  mode->completed(px4_ros2::Result::Success);  // modes may call it repeatedly
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  EXPECT_TRUE(fmu.configs.empty());

  // Activated again: the configured timeout again
  fake_autopilot.setModeAndArm(0);
  ASSERT_TRUE(waitFor(node, [&mode] { return !mode->isActive(); }));
  mode->setSendSetpoints(true);
  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitFor(node, [&fmu] { return !fmu.configs.empty(); }));
  EXPECT_EQ(fmu.configs.front().timeout_ms, 150);
}

TEST(modes, setpointTimeoutDefaultCompleted)
{
  // Without a setpoint timeout completed() sends no configuration
  auto node = std::make_shared<rclcpp::Node>("test_node");
  FakeAutopilot fake_autopilot(node, "");
  SetpointConfigResponder& fmu = fake_autopilot.setpointConfigResponder();
  auto mode = std::make_shared<SetpointTestMode>(*node, px4_ros2::ModeBase::Settings{"test"});
  ASSERT_TRUE(mode->doRegister());

  fake_autopilot.setModeAndArm(mode->id());
  ASSERT_TRUE(waitFor(node, [&fmu] { return !fmu.configs.empty(); }));
  ASSERT_TRUE(waitForUpdates(node, *mode, 10));
  fmu.configs.clear();
  mode->setSendSetpoints(false);
  mode->completed(px4_ros2::Result::Success);
  ASSERT_TRUE(waitForUpdates(node, *mode, 20));
  EXPECT_TRUE(fmu.configs.empty());
}
