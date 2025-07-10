/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once
#include <gtest/gtest.h>
#include <px4_ros2/mission/trajectory/trajectory_executor.hpp>
#include "fake_autopilot.hpp"
#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/visit.hpp>

class TrajectoryExecutorTest : public px4_ros2::TrajectoryExecutorInterface
{
public:
  explicit TrajectoryExecutorTest(px4_ros2::ModeBase & mode, bool verbose_settings = false)
  : _node(mode.node()), _verbose_settings(verbose_settings)
  {
    // We need to have some setpoint
    _setpoint = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(mode);
    // Set very high update rate for tests
    mode.setSetpointUpdateRate(1000.f);
  }
  ~TrajectoryExecutorTest() override = default;

  bool navigationItemTypeSupported(px4_ros2::NavigationItemType type) override {return true;}
  bool frameSupported(px4_ros2::MissionFrame frame) override {return true;}
  void runTrajectory(const TrajectoryConfig & config) override
  {
    _current_config = config;
    _index = config.start_index;
    _has_config = true;
  }
  void updateSetpoint() override
  {
    EXPECT_TRUE(_has_config);
    EXPECT_LE(_index, _current_config.end_index);
    if (const auto * navigation_item =
      std::get_if<px4_ros2::NavigationItem>(&_current_config.trajectory->items()[_index]))
    {
      std::visit(
        px4_ros2::util::Overloaded{
        [&](const px4_ros2::Waypoint & waypoint)
        {
          RCLCPP_INFO(
            _node.get_logger(), "Waypoint[%i]: %.6f %.6f %.3f, frame: %i", _index,
            waypoint.coordinate(0), waypoint.coordinate(1), waypoint.coordinate(2),
            static_cast<int>(waypoint.frame));
          if (_verbose_settings) {
            RCLCPP_INFO(
              _node.get_logger(),
              "       With: hor v=%.1f m/s, vert v=%.1f m/s, yaw rate=%.1f deg/s",
              _current_config.options.horizontal_velocity.value_or(NAN),
              _current_config.options.vertical_velocity.value_or(NAN),
              px4_ros2::radToDeg(_current_config.options.max_heading_rate.value_or(NAN)));
          }
        }
      }, navigation_item->data);

    } else {
      EXPECT_TRUE(false) << "Invalid mission item (action?)";
    }
    const int index = _index;
    ++_index;
    _current_config.on_index_reached(index);
  }

private:
  bool _has_config{false};
  TrajectoryConfig _current_config;
  int _index{};
  std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _setpoint;
  rclcpp::Node & _node;
  const bool _verbose_settings;
};

class MissionExecutorTest : public px4_ros2::MissionExecutor
{
public:
  static constexpr const char * kTopicPrefix = "/test/";

  MissionExecutorTest(
    const std::string & mode_name, const Configuration & configuration,
    rclcpp::Node & node, std::shared_ptr<FakeAutopilot> fake_autopilot)
  : MissionExecutor(mode_name, configuration, node, kTopicPrefix), _node(node), _fake_autopilot(
      std::move(fake_autopilot))
  {
    setCommandHandler(
      [this](uint32_t command, float param1)
      {
        if (command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE) {
          if (mode_change_request) {
            const int nav_state = std::round(param1);
            return mode_change_request(nav_state);
          }
        } else if (command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF) {
          if (mode_change_request) {
            return mode_change_request(px4_ros2::ModeBase::kModeIDTakeoff);
          }
        }

        RCLCPP_DEBUG(_node.get_logger(), "Handling command %i (accepting)", command);
        return true;
      });
    _land_detected->onUpdate(
      [this](const px4_msgs::msg::VehicleLandDetected & landed) {
        onLanded(landed);
      });

    // Set a default for changing modes
    mode_change_request = [this](uint8_t nav_state)
      {
        const bool send_completed = nav_state != modeId() &&
          nav_state != px4_ros2::ModeBase::kModeIDLoiter;
        setModeAndArm(nav_state, send_completed);
        return true;
      };
  }

  std::shared_ptr<px4_ros2::LandDetected> landDetected() const {return _land_detected;}

  ~MissionExecutorTest() override = default;

  px4_ros2::ModeBase::ModeID modeId() const {return _mode_id;}
  int modeExecutorId() const {return _mode_executor_id;}
  void setModeAndArm(uint8_t mode_id, bool send_completed = true)
  {
    _fake_autopilot->setModeAndArm(mode_id, _mode_executor_id);

    if (send_completed) {
      if (mode_id == px4_ros2::ModeBase::kModeIDRtl || mode_id == px4_ros2::ModeBase::kModeIDLand) {
        _mode_continue_state = ModeContinueState::Landing;
      } else if (mode_id == px4_ros2::ModeBase::kModeIDTakeoff) {
        _mode_continue_state = ModeContinueState::NotLanding;
      } else {
        _mode_continue_state = ModeContinueState::Continue;
      }
      _mode_continue_timer = _node.create_wall_timer(
        std::chrono::milliseconds(10), [this]()
        {
          handleModeContinueState();
        });
    } else {
      _mode_continue_state = ModeContinueState::Idle;
    }
  }


  std::function<bool(int)> mode_change_request;

protected:
  bool doRegisterImpl(MissionMode & mode, MissionModeExecutor & executor) override
  {
    mode.disableWatchdogTimer();
    executor.setRegistration(std::make_shared<FakeRegistration>(_node));
    const bool ret = MissionExecutor::doRegisterImpl(mode, executor);
    _mode_id = executor.ownedMode().id();
    _mode_executor_id = executor.id();
    return ret;
  }

private:
  void handleModeContinueState()
  {
    switch (_mode_continue_state) {
      case ModeContinueState::Idle:
        break;
      case ModeContinueState::Landing:
        _fake_autopilot->setLanded(true);
        _mode_continue_state = ModeContinueState::Continue;
        break;
      case ModeContinueState::NotLanding:
        _fake_autopilot->setLanded(false);
        _mode_continue_state = ModeContinueState::Continue;
        break;
      case ModeContinueState::Continue:
        _fake_autopilot->sendModeCompleted();
        _mode_continue_state = ModeContinueState::Idle;
        break;
    }
    _mode_continue_timer.reset();
  }

  enum class ModeContinueState
  {
    Idle,
    Landing,
    NotLanding,
    Continue,
  };
  void onLanded(const px4_msgs::msg::VehicleLandDetected & landed)
  {
    handleModeContinueState();
  }
  rclcpp::Node & _node;
  px4_ros2::ModeBase::ModeID _mode_id{};
  uint8_t _mode_executor_id{};
  std::shared_ptr<FakeAutopilot> _fake_autopilot;
  rclcpp::TimerBase::SharedPtr _mode_continue_timer;
  ModeContinueState _mode_continue_state{ModeContinueState::Idle};
};

class MissionExecutionTester : public testing::Test
{
public:
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<RosLogCapture> log_capture;
  std::shared_ptr<FakeAutopilot> fake_autopilot;
  const std::string persistence_file = "/tmp/mission_execution_test_persistence.tmp";

protected:
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_node");
    log_capture = std::make_unique<RosLogCapture>(node);
    fake_autopilot = std::make_shared<FakeAutopilot>(node, MissionExecutorTest::kTopicPrefix);

    std::remove(persistence_file.c_str());
  }
};
