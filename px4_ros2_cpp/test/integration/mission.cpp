/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>

#include "util.hpp"

#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <px4_ros2/utils/geodesic.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

static const std::string kName = "Test Mission Mode";

namespace mission_tests
{

class MissionExecutorTest : public px4_ros2::MissionExecutor
{
public:
  using MissionExecutor::MissionExecutor;
  px4_ros2::ModeBase::ModeID getModeId() const {return modeId();}
  px4_ros2::ModeExecutorBase & getModeExecutor() {return modeExecutor();}
};

class TestMission
{
public:
  explicit TestMission(rclcpp::Node & node)
  : _node(node), _vehicle_state(node) {}

  void run();

private:
  enum class State
  {
    SwitchMode,
    WaitingForSwitch,
    WaitForArming,
    Arming,
  };

  State _state{State::SwitchMode};

  rclcpp::Node & _node;
  rclcpp::TimerBase::SharedPtr _test_timeout;

  std::unique_ptr<MissionExecutorTest> _mission;
  VehicleState _vehicle_state;
};

void TestMission::run()
{
  _test_timeout = _node.create_wall_timer(
    80s, [] {
      EXPECT_TRUE(false) << "Timeout";
      rclcpp::shutdown();
    });

  _mission = std::make_unique<MissionExecutorTest>(
    kName,
    px4_ros2::MissionExecutor::Configuration(), _node);
  ASSERT_TRUE(_mission->doRegister());
  px4_ros2::Context context(_node);
  px4_ros2::OdometryGlobalPosition global_position(context);

  // Testing steps:
  // - Switch into the mission mode
  // - Wait until we can arm
  // - arm
  // - at this point the mission will be executed
  // - when completed, check that all waypoints have been reached

  const std::vector<px4_ros2::Waypoint> waypoints{
    px4_ros2::Waypoint({47.3977419, 8.5455939, 500}),
    px4_ros2::Waypoint({47.39791657, 8.54595214, 504}),
    px4_ros2::Waypoint({47.39820919, 8.5457699, 500})
  };
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      waypoints[0], waypoints[1], waypoints[2],
      px4_ros2::ActionItem("land")});
  _mission->setMission(mission);
  std::vector<bool> waypoints_reached(waypoints.size(), false);

  const float acceptance_radius = 2.0f;
  global_position.onUpdate(
    [&](const px4_msgs::msg::VehicleGlobalPosition & msg)
    {
      for (std::size_t i = 0; i < waypoints.size(); i++) {
        if (waypoints_reached[i]) {
          continue;
        }
        if (px4_ros2::distanceToGlobalPosition(
          global_position.position(),
          waypoints[i].coordinate) < acceptance_radius)
        {
          RCLCPP_INFO(_node.get_logger(), "Waypoint %zu reached", i);
          waypoints_reached[i] = true;
        }
      }
    });

  _vehicle_state.setOnVehicleStatusUpdate(
    [this](const px4_msgs::msg::VehicleStatus::UniquePtr & msg) {
      if (_state == State::SwitchMode) {

        _vehicle_state.callbackOnModeSet(
          [this]() {
            RCLCPP_INFO(_node.get_logger(), "Mode Activated, waiting for arming");
            _state = State::WaitForArming;
          }, _mission->getModeId());

        _vehicle_state.sendCommand(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE,
          _mission->getModeId());
        _state = State::WaitingForSwitch;
      }

      if (_state == State::WaitForArming) {
        if (msg->pre_flight_checks_pass) {
          RCLCPP_INFO(_node.get_logger(), "Arming possible, arming");
          _state = State::Arming;
          _mission->getModeExecutor().arm(
            [](px4_ros2::Result result) {
              ASSERT_EQ(result, px4_ros2::Result::Success);
            });
        }
      }
    });

  _mission->onCompleted(
    [&]
    {
      for (std::size_t i = 0; i < waypoints_reached.size(); i++) {
        EXPECT_TRUE(waypoints_reached[i]) << "Waypoint " << i << " not reached";
      }
      rclcpp::shutdown();
    });

  rclcpp::spin(_node.shared_from_this());
}

TEST_F(ModesTest, runMission)
{
  auto test_node = initNode();
  ASSERT_TRUE(px4_ros2::waitForFMU(*test_node, 10s));
  TestMission test{*test_node};
  test.run();
}

} // namespace mission_tests
