/****************************************************************************
* Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "fake_registration.hpp"
#include "utils/ros_log_capture.hpp"
#include "utils/wait_for.hpp"
#include "mission_execution.hpp"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <px4_ros2/third_party/nlohmann/json.hpp>
#include <px4_ros2/mission/actions/action.hpp>

TEST(MissionExecutor, json)
{
  const auto * const json_str =
    R"({
    "mission": {
        "defaults": {
            "horizontalVelocity": 12.5,
            "maxHeadingRate": 60.000003814697266,
            "verticalVelocity": 3.0
        },
        "items": [
            {
                "frame": "global",
                "id": "item id 0",
                "navigationType": "waypoint",
                "type": "navigation",
                "x": 123.0,
                "y": 111.0,
                "z": 11.02
            },
            {
                "frame": "global",
                "navigationType": "waypoint",
                "type": "navigation",
                "x": 124.0,
                "y": 222.0,
                "z": 0.0
            },
            {
                "arg1": 4372,
                "arg2": [
                    1,
                    2,
                    3
                ],
                "arg3": false,
                "id": "custom action item id 1",
                "type": "custom_action"
            },
            {
                "frame": "global",
                "id": "item id 2",
                "navigationType": "waypoint",
                "type": "navigation",
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            },
            {
                "type": "rtl"
            }
        ]
    },
    "version": 1
})";
  const auto mission = nlohmann::json::parse(json_str).get<px4_ros2::Mission>();
  EXPECT_FALSE(mission.checksum().empty());
  EXPECT_TRUE(
    fabsf(
      mission.defaults().trajectory_options.horizontal_velocity.value() - 12.5f) < 0.00001f);
  ASSERT_EQ(mission.items().size(), 5U);
  const nlohmann::json mission_json = mission;
  const std::string mission_str = mission_json.dump(4);
  EXPECT_EQ(json_str, mission_str);
}

TEST(MissionExecutor, jsonError)
{
  const auto * const json_str =
    R"(
  {
    "version": 1,
    "mission": {
        "items": [
           {
               "type": "navigation",
               "navigationType": "invalid"
           }
        ]
    }
  }
)";
  EXPECT_THROW(nlohmann::json::parse(json_str).get<px4_ros2::Mission>(), nlohmann::json::exception);
}

TEST(MissionExecutor, fileMonitor)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  constexpr auto kFilename = "/tmp/test_mission.json";
  if (std::filesystem::exists(kFilename)) {
    std::filesystem::remove(kFilename);
  }

  std::shared_ptr<px4_ros2::Mission> mission;
  const px4_ros2::MissionFileMonitor monitor(node, kFilename,
    [&](const std::shared_ptr<px4_ros2::Mission> & m)
    {
      mission = m;
    });

  // Create JSON file
  const auto * const json_str =
    R"({
    "mission": {
        "items": [
            {
                "frame": "global",
                "navigationType": "waypoint",
                "type": "navigation",
                "x": 123.0,
                "y": 111.0,
                "z": 11.02
            },
            {
                "type": "rtl"
            }
        ]
    },
    "version": 1
})";
  std::ofstream file(kFilename);
  file << json_str;
  file.close();

  ASSERT_TRUE(waitFor(node, [&] {return mission != nullptr;}));
  EXPECT_EQ(mission->items().size(), 2U);
}

TEST_F(MissionExecutionTester, basicMission)
{
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
      px4_ros2::Waypoint({47.39810924, 8.54598000, 501}),
      px4_ros2::Waypoint({47.39920293, 8.54395194, 505}),
      px4_ros2::ActionItem("land")});

  MissionExecutorTest mission_executor("my mission",
    px4_ros2::MissionExecutor::Configuration()
    .withTrajectoryExecutor<TrajectoryExecutorTest>(),
    *node, fake_autopilot);
  ASSERT_TRUE(mission_executor.doRegister());
  mission_executor.setMission(mission);

  bool mission_completed{false};
  mission_executor.onCompleted([&mission_completed] {mission_completed = true;});

  mission_executor.setModeAndArm(mission_executor.modeId());

  // Wait for the mission to complete
  EXPECT_TRUE(waitFor(node, [&mission_completed] {return mission_completed;}));


  log_capture->expectEqual(
    R"V0G0N(
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running trajectory (start: 1, end: 3, stop: 1)
Running mode 100
FakeAutopilot: setting mode (id=100)
Mode 'my mission' activated
Waypoint[1]: 47.398209 8.545952 500.000, frame: 0
Setting current mission item to 2
Waypoint[2]: 47.398109 8.545980 501.000, frame: 0
Setting current mission item to 3
Waypoint[3]: 47.399203 8.543952 505.000, frame: 0
Setting current mission item to 4
Running action 'land'
Running mode 18
FakeAutopilot: setting mode (id=18)
Mode 'my mission' deactivated
FakeAutopilot: setting landed to 1
FakeAutopilot: sending mode completed (id=18)
Got matching ModeCompleted message, result: 0
Setting current mission item to 5
Mission completed
)V0G0N");
  log_capture->checkHasNoErrors();
}

class CustomActionTest : public px4_ros2::ActionInterface
{
public:
  explicit CustomActionTest(px4_ros2::ModeBase & mode) {}
  std::string name() const override {return "customAction";}

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    EXPECT_EQ(arguments.at<int>("customArgument"), 1234);
    on_completed();
  }
};
TEST_F(MissionExecutionTester, customAction)
{
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
      px4_ros2::ActionItem("customAction", px4_ros2::ActionArguments{{{"customArgument", 1234}}}),
      px4_ros2::ActionItem("rtl")});

  MissionExecutorTest mission_executor("my mission",
    px4_ros2::MissionExecutor::Configuration()
    .addCustomAction<CustomActionTest>()
    .withTrajectoryExecutor<TrajectoryExecutorTest>(),
    *node, fake_autopilot);
  ASSERT_TRUE(mission_executor.doRegister());
  mission_executor.setMission(mission);

  bool mission_completed{false};
  mission_executor.onCompleted([&mission_completed] {mission_completed = true;});

  mission_executor.setModeAndArm(mission_executor.modeId());

  // Wait for the mission to complete
  EXPECT_TRUE(waitFor(node, [&mission_completed] {return mission_completed;}));


  log_capture->expectEqual(
    R"V0G0N(
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running trajectory (start: 1, end: 1, stop: 1)
Running mode 100
FakeAutopilot: setting mode (id=100)
Mode 'my mission' activated
Waypoint[1]: 47.398209 8.545952 500.000, frame: 0
Setting current mission item to 2
Running action 'customAction'
Setting current mission item to 3
Running action 'rtl'
Running mode 5
FakeAutopilot: setting mode (id=5)
Mode 'my mission' deactivated
FakeAutopilot: setting landed to 1
FakeAutopilot: sending mode completed (id=5)
Got matching ModeCompleted message, result: 0
Setting current mission item to 4
Mission completed
)V0G0N");
  log_capture->checkHasNoErrors();
}


class CustomActionTrajectoryTest : public px4_ros2::ActionInterface
{
public:
  explicit CustomActionTrajectoryTest(px4_ros2::ModeBase & mode) {}
  std::string name() const override {return "customActionTrajectory";}

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    // Run a custom trajectory
    auto mission = std::make_shared<px4_ros2::Mission>(
      std::vector<px4_ros2::MissionItem>{px4_ros2::Waypoint({47.39820919, 8.54595214, 400}),
        px4_ros2::Waypoint({47.39820919, 8.54595214, 410})});
    handler->runTrajectory(mission, on_completed);
  }
};
TEST_F(MissionExecutionTester, customActionTrajectory)
{
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      px4_ros2::ActionItem("customActionTrajectory", {}),
      px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
      px4_ros2::ActionItem("rtl")});

  MissionExecutorTest mission_executor("my mission",
    px4_ros2::MissionExecutor::Configuration()
    .addCustomAction<CustomActionTrajectoryTest>()
    .withTrajectoryExecutor<TrajectoryExecutorTest>(),
    *node, fake_autopilot);
  ASSERT_TRUE(mission_executor.doRegister());
  mission_executor.setMission(mission);

  bool mission_completed{false};
  mission_executor.onCompleted([&mission_completed] {mission_completed = true;});

  mission_executor.setModeAndArm(mission_executor.modeId());

  // Wait for the mission to complete
  EXPECT_TRUE(waitFor(node, [&mission_completed] {return mission_completed;}));


  log_capture->expectEqual(
    R"V0G0N(
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running action 'customActionTrajectory'
Running custom trajectory (2 items)
Running trajectory (start: 0, end: 1, stop: 1)
Running mode 100
FakeAutopilot: setting mode (id=100)
Mode 'my mission' activated
Waypoint[0]: 47.398209 8.545952 400.000, frame: 0
Waypoint[1]: 47.398209 8.545952 410.000, frame: 0
Setting current mission item to 2
Running trajectory (start: 2, end: 2, stop: 0)
Waypoint[2]: 47.398209 8.545952 500.000, frame: 0
Setting current mission item to 3
Running action 'rtl'
Running mode 5
FakeAutopilot: setting mode (id=5)
Mode 'my mission' deactivated
FakeAutopilot: setting landed to 1
FakeAutopilot: sending mode completed (id=5)
Got matching ModeCompleted message, result: 0
Setting current mission item to 4
Mission completed
)V0G0N");
  log_capture->checkHasNoErrors();
}


class CustomActionInterruptible : public px4_ros2::ActionInterface
{
public:
  CustomActionInterruptible(
    px4_ros2::ModeBase & mode,
    std::function<bool(const px4_ros2::ActionArguments &)> on_run, int & num_run_calls,
    bool supports_resume_from_landed = false,
    std::string name = "customActionInterruptible")
  : _on_run(std::move(on_run)), _num_run_calls(num_run_calls), _supports_resume_from_landed(
      supports_resume_from_landed), _name(std::move(name)) {}
  std::string name() const override {return _name;}

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    ++_num_run_calls;
    if (_on_run(arguments)) {
      on_completed();
    }
  }

  bool supportsResumeFromLanded() override {return _supports_resume_from_landed;}

private:
  std::function<bool(const px4_ros2::ActionArguments &)> _on_run;
  int & _num_run_calls;
  const bool _supports_resume_from_landed;
  const std::string _name;
};

TEST_F(MissionExecutionTester, resumeWithoutResumeAction)
{
  // Test resuming a mission without 'onResume' action
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      px4_ros2::ActionItem("customActionInterruptible", {}),
      px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
      px4_ros2::ActionItem("rtl")});

  bool should_interrupt = true;
  bool wait_for_deactivation = false;
  int num_run_calls{0};
  auto config = px4_ros2::MissionExecutor::Configuration();
  config.default_actions = {"rtl", "takeoff"}; // Disable default actions except rtl and takeoff
  const auto on_action_run = [&](const px4_ros2::ActionArguments & arguments)
    {
      if (should_interrupt) {
        // When the action is first activated, switch to position control, which deactivates the mission execution.
        // Afterwards, switch back to the mission to finish it.
        fake_autopilot->setModeAndArm(px4_ros2::ModeBase::kModeIDPosctl, 0);
        should_interrupt = false;
        wait_for_deactivation = true;
        EXPECT_FALSE(arguments.resuming());
        return false;
      }
      EXPECT_TRUE(arguments.resuming());
      return true;
    };
  MissionExecutorTest mission_executor("my mission",
    config
    .addCustomAction<CustomActionInterruptible>(on_action_run, num_run_calls)
    .withTrajectoryExecutor<TrajectoryExecutorTest>(),
    *node, fake_autopilot);
  ASSERT_TRUE(mission_executor.doRegister());
  mission_executor.setMission(mission);

  bool mission_completed{false};
  mission_executor.onCompleted([&mission_completed] {mission_completed = true;});

  mission_executor.onDeactivated(
    [&] {
      EXPECT_TRUE(wait_for_deactivation);
      wait_for_deactivation = false;
      mission_executor.setModeAndArm(mission_executor.modeId());
    });

  mission_executor.setModeAndArm(mission_executor.modeId());

  // Wait for the mission to complete
  EXPECT_TRUE(waitFor(node, [&mission_completed] {return mission_completed;}));

  EXPECT_EQ(num_run_calls, 2);

  log_capture->expectEqual(
    R"V0G0N(
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running action 'customActionInterruptible'
FakeAutopilot: setting mode (id=2)
Mode executor 'my mission' deactivated (1)
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 1
Running action 'customActionInterruptible'
Setting current mission item to 2
Running trajectory (start: 2, end: 2, stop: 0)
Mode 'my mission' activated
Waypoint[2]: 47.398209 8.545952 500.000, frame: 0
Setting current mission item to 3
Running action 'rtl'
Running mode 5
FakeAutopilot: setting mode (id=5)
Mode 'my mission' deactivated
FakeAutopilot: setting landed to 1
FakeAutopilot: sending mode completed (id=5)
Got matching ModeCompleted message, result: 0
Setting current mission item to 4
Mission completed
)V0G0N");
  log_capture->checkHasNoErrors();
}

TEST_F(MissionExecutionTester, resumeInAir)
{
  // Test resuming a mission in-air
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      px4_ros2::ActionItem("customActionInterruptible", {}),
      px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
      px4_ros2::ActionItem("rtl")});

  bool should_interrupt = true;
  bool wait_for_deactivation = false;
  int num_run_calls{0};
  const auto on_action_run = [&](const px4_ros2::ActionArguments & arguments)
    {
      if (should_interrupt) {
        // When the action is first activated, switch to position control, which deactivates the mission execution.
        // Afterwards, switch back to the mission to finish it.
        fake_autopilot->setModeAndArm(px4_ros2::ModeBase::kModeIDPosctl, 0);
        should_interrupt = false;
        wait_for_deactivation = true;
        EXPECT_FALSE(arguments.resuming());
        return false;
      }
      EXPECT_TRUE(arguments.resuming());
      return true;
    };
  MissionExecutorTest mission_executor("my mission",
    px4_ros2::MissionExecutor::Configuration()
    .addCustomAction<CustomActionInterruptible>(on_action_run, num_run_calls)
    .withTrajectoryExecutor<TrajectoryExecutorTest>(),
    *node, fake_autopilot);
  ASSERT_TRUE(mission_executor.doRegister());
  mission_executor.setMission(mission);

  bool mission_completed{false};
  mission_executor.onCompleted([&mission_completed] {mission_completed = true;});

  mission_executor.onDeactivated(
    [&] {
      EXPECT_TRUE(wait_for_deactivation);
      wait_for_deactivation = false;
      mission_executor.setModeAndArm(mission_executor.modeId());
    });

  mission_executor.setModeAndArm(mission_executor.modeId());

  // Wait for the mission to complete
  EXPECT_TRUE(waitFor(node, [&mission_completed] {return mission_completed;}));

  EXPECT_EQ(num_run_calls, 2);

  log_capture->expectEqual(
    R"V0G0N(
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running action 'customActionInterruptible'
FakeAutopilot: setting mode (id=2)
Mode executor 'my mission' deactivated (1)
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 1
Running action 'onResume'
Running action 'customActionInterruptible'
Setting current mission item to 2
Running trajectory (start: 2, end: 2, stop: 0)
Mode 'my mission' activated
Waypoint[2]: 47.398209 8.545952 500.000, frame: 0
Setting current mission item to 3
Running action 'rtl'
Running mode 5
FakeAutopilot: setting mode (id=5)
Mode 'my mission' deactivated
FakeAutopilot: setting landed to 1
FakeAutopilot: sending mode completed (id=5)
Got matching ModeCompleted message, result: 0
Setting current mission item to 4
Mission completed
)V0G0N");
  log_capture->checkHasNoErrors();
}

TEST_F(MissionExecutionTester, resumeFromLanded)
{
  // Test resuming a mission from landed state
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      px4_ros2::Waypoint({47.34820919, 8.54395214, 503}),
      px4_ros2::Waypoint({47.34720919, 8.54385214, 502}),
      px4_ros2::Waypoint({47.34620919, 8.54375214, 501}),
      px4_ros2::ActionItem("customActionInterruptible", {}),
      px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
      px4_ros2::ActionItem("rtl")});

  bool should_interrupt = true;
  bool wait_for_deactivation = false;
  int num_run_calls{0};
  const auto on_action_run = [&](const px4_ros2::ActionArguments & arguments)
    {
      if (should_interrupt) {
        // When the action is first activated, switch to position control, which deactivates the mission execution.
        // Afterwards, switch back to the mission to finish it.
        fake_autopilot->setLanded(true);
        fake_autopilot->setModeAndArm(px4_ros2::ModeBase::kModeIDPosctl, 0);
        should_interrupt = false;
        wait_for_deactivation = true;
        return false;
      }
      return true;
    };
  MissionExecutorTest mission_executor("my mission",
    px4_ros2::MissionExecutor::Configuration()
    .addCustomAction<CustomActionInterruptible>(on_action_run, num_run_calls)
    .withTrajectoryExecutor<TrajectoryExecutorTest>(),
    *node, fake_autopilot);
  ASSERT_TRUE(mission_executor.doRegister());
  mission_executor.setMission(mission);

  bool mission_completed{false};
  mission_executor.onCompleted([&mission_completed] {mission_completed = true;});

  rclcpp::TimerBase::SharedPtr timer;
  mission_executor.onDeactivated(
    [&] {
      EXPECT_TRUE(wait_for_deactivation);
      wait_for_deactivation = false;
      // Switch back to the mission, but async to avoid reordering of events
      timer = node->create_wall_timer(
        1ms, [&]
        {
          timer->cancel();
          mission_executor.setModeAndArm(mission_executor.modeId());
        });
    });

  mission_executor.setModeAndArm(mission_executor.modeId());

  // Wait for the mission to complete
  EXPECT_TRUE(waitFor(node, [&mission_completed] {return mission_completed;}));

  EXPECT_EQ(num_run_calls, 2);

  log_capture->expectEqual(
    R"V0G0N(
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running trajectory (start: 1, end: 3, stop: 1)
Running mode 100
FakeAutopilot: setting mode (id=100)
Mode 'my mission' activated
Waypoint[1]: 47.348209 8.543952 503.000, frame: 0
Setting current mission item to 2
Waypoint[2]: 47.347209 8.543852 502.000, frame: 0
Setting current mission item to 3
Waypoint[3]: 47.346209 8.543752 501.000, frame: 0
Setting current mission item to 4
Running action 'customActionInterruptible'
FakeAutopilot: setting landed to 1
FakeAutopilot: setting mode (id=2)
Mode executor 'my mission' deactivated (1)
Mode 'my mission' deactivated
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 4
Running action 'onResume'
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Resume: navigating to waypoint before the current one
Running custom trajectory (1 items)
Running trajectory (start: 0, end: 0, stop: 0)
Running mode 100
FakeAutopilot: setting mode (id=100)
Mode 'my mission' activated
Waypoint[0]: 47.346209 8.543752 501.000, frame: 0
Running action 'customActionInterruptible'
Setting current mission item to 5
Running trajectory (start: 5, end: 5, stop: 0)
Waypoint[5]: 47.398209 8.545952 500.000, frame: 0
Setting current mission item to 6
Running action 'rtl'
Running mode 5
FakeAutopilot: setting mode (id=5)
Mode 'my mission' deactivated
FakeAutopilot: setting landed to 1
FakeAutopilot: sending mode completed (id=5)
Got matching ModeCompleted message, result: 0
Setting current mission item to 7
Mission completed
)V0G0N");
  log_capture->checkHasNoErrors();
}

TEST_F(MissionExecutionTester, resumeFromLandedInRtl)
{
  // Test resuming a mission from landed state when interrupted in rtl.
  // The mission should then resume from the beginning, as rtl was the last item.
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      px4_ros2::Waypoint({47.34820919, 8.54395214, 503}),
      px4_ros2::Waypoint({47.34720919, 8.54385214, 502}),
      px4_ros2::Waypoint({47.34620919, 8.54375214, 501}),
      px4_ros2::ActionItem("rtl", {})});

  bool should_interrupt = true;
  bool wait_for_deactivation = false;
  int num_run_calls{0};
  MissionExecutorTest * mission_executor_test{nullptr};
  const auto on_action_run = [&](const px4_ros2::ActionArguments & arguments)
    {
      if (should_interrupt) {
        // When the action is first activated, switch to position control, which deactivates the mission execution.
        // Afterwards, switch back to the mission to finish it.

        // Wait for the landed state update before continuing to avoid race conditions
        EXPECT_TRUE(mission_executor_test);
        if (mission_executor_test) {
          mission_executor_test->landDetected()->onUpdate(
            [&](const px4_msgs::msg::VehicleLandDetected & landed)
            {
              if (landed.landed) {
                fake_autopilot->setModeAndArm(px4_ros2::ModeBase::kModeIDPosctl, 0);
                should_interrupt = false;
                wait_for_deactivation = true;
              }
            });
        }
        fake_autopilot->setLanded(true);
        return false;
      }
      return true;
    };
  const bool supports_resume_from_landed = false;
  const std::string action_name = "rtl";
  MissionExecutorTest mission_executor("my mission",
    px4_ros2::MissionExecutor::Configuration()
    .addCustomAction<CustomActionInterruptible>(
      on_action_run, num_run_calls,
      supports_resume_from_landed, action_name)
    .withTrajectoryExecutor<TrajectoryExecutorTest>(),
    *node, fake_autopilot);
  mission_executor_test = &mission_executor;
  ASSERT_TRUE(mission_executor.doRegister());
  mission_executor.setMission(mission);

  bool mission_completed{false};
  mission_executor.onCompleted([&mission_completed] {mission_completed = true;});

  rclcpp::TimerBase::SharedPtr timer;
  mission_executor.onDeactivated(
    [&] {
      EXPECT_TRUE(wait_for_deactivation);
      wait_for_deactivation = false;
      // Switch back to the mission, but async to avoid reordering of events
      timer = node->create_wall_timer(
        1ms, [&]
        {
          timer->cancel();
          mission_executor.setModeAndArm(mission_executor.modeId());
        });
    });

  mission_executor.setModeAndArm(mission_executor.modeId());

  // Wait for the mission to complete
  EXPECT_TRUE(waitFor(node, [&mission_completed] {return mission_completed;}));

  EXPECT_EQ(num_run_calls, 2);

  log_capture->expectEqual(
    R"V0G0N(
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running trajectory (start: 1, end: 3, stop: 1)
Running mode 100
FakeAutopilot: setting mode (id=100)
Mode 'my mission' activated
Waypoint[1]: 47.348209 8.543952 503.000, frame: 0
Setting current mission item to 2
Waypoint[2]: 47.347209 8.543852 502.000, frame: 0
Setting current mission item to 3
Waypoint[3]: 47.346209 8.543752 501.000, frame: 0
Setting current mission item to 4
Running action 'rtl'
FakeAutopilot: setting landed to 1
FakeAutopilot: setting mode (id=2)
Mode executor 'my mission' deactivated (1)
Mode 'my mission' deactivated
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running trajectory (start: 1, end: 3, stop: 1)
Running mode 100
FakeAutopilot: setting mode (id=100)
Mode 'my mission' activated
Waypoint[1]: 47.348209 8.543952 503.000, frame: 0
Setting current mission item to 2
Waypoint[2]: 47.347209 8.543852 502.000, frame: 0
Setting current mission item to 3
Waypoint[3]: 47.346209 8.543752 501.000, frame: 0
Setting current mission item to 4
Running action 'rtl'
Setting current mission item to 5
Mission completed, entering Hold
Running mode 4
FakeAutopilot: setting mode (id=4)
Mode 'my mission' deactivated
)V0G0N");
  log_capture->checkHasNoErrors();
}

TEST_F(MissionExecutionTester, resumeFromLandedWithResumeSupport)
{
  // Test resuming a mission from landed state with a custom action that supports resuming from landed
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      px4_ros2::Waypoint({47.34820919, 8.54395214, 503}),
      px4_ros2::Waypoint({47.34720919, 8.54385214, 502}),
      px4_ros2::Waypoint({47.34620919, 8.54375214, 501}),
      px4_ros2::ActionItem("customActionInterruptible", {}),
      px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
      px4_ros2::ActionItem("rtl")});

  bool should_interrupt = true;
  bool wait_for_deactivation = false;
  int num_run_calls{0};
  const auto on_action_run = [&](const px4_ros2::ActionArguments & arguments)
    {
      if (should_interrupt) {
        // When the action is first activated, switch to position control, which deactivates the mission execution.
        // Afterwards, switch back to the mission to finish it.
        fake_autopilot->setLanded(true);
        fake_autopilot->setModeAndArm(px4_ros2::ModeBase::kModeIDPosctl, 0);
        should_interrupt = false;
        wait_for_deactivation = true;
        EXPECT_FALSE(arguments.resuming());
        return false;
      }
      EXPECT_TRUE(arguments.resuming());
      return true;
    };
  const bool supports_resume_from_landed = true;
  MissionExecutorTest mission_executor("my mission",
    px4_ros2::MissionExecutor::Configuration()
    .addCustomAction<CustomActionInterruptible>(
      on_action_run, num_run_calls,
      supports_resume_from_landed)
    .withTrajectoryExecutor<TrajectoryExecutorTest>(),
    *node, fake_autopilot);
  ASSERT_TRUE(mission_executor.doRegister());
  mission_executor.setMission(mission);

  bool mission_completed{false};
  mission_executor.onCompleted([&mission_completed] {mission_completed = true;});

  rclcpp::TimerBase::SharedPtr timer;
  mission_executor.onDeactivated(
    [&] {
      EXPECT_TRUE(wait_for_deactivation);
      wait_for_deactivation = false;
      // Switch back to the mission, but async to avoid reordering of events
      timer = node->create_wall_timer(
        1ms, [&]
        {
          timer->cancel();
          mission_executor.setModeAndArm(mission_executor.modeId());
        });
    });

  mission_executor.setModeAndArm(mission_executor.modeId());

  // Wait for the mission to complete
  EXPECT_TRUE(waitFor(node, [&mission_completed] {return mission_completed;}));

  EXPECT_EQ(num_run_calls, 2);

  log_capture->expectEqual(
    R"V0G0N(
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running trajectory (start: 1, end: 3, stop: 1)
Running mode 100
FakeAutopilot: setting mode (id=100)
Mode 'my mission' activated
Waypoint[1]: 47.348209 8.543952 503.000, frame: 0
Setting current mission item to 2
Waypoint[2]: 47.347209 8.543852 502.000, frame: 0
Setting current mission item to 3
Waypoint[3]: 47.346209 8.543752 501.000, frame: 0
Setting current mission item to 4
Running action 'customActionInterruptible'
FakeAutopilot: setting landed to 1
FakeAutopilot: setting mode (id=2)
Mode executor 'my mission' deactivated (1)
Mode 'my mission' deactivated
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 4
Running action 'onResume'
Resume: current action supports resuming from landed, continuing directly
Running action 'customActionInterruptible'
Setting current mission item to 5
Running trajectory (start: 5, end: 5, stop: 0)
Mode 'my mission' activated
Waypoint[5]: 47.398209 8.545952 500.000, frame: 0
Setting current mission item to 6
Running action 'rtl'
Running mode 5
FakeAutopilot: setting mode (id=5)
Mode 'my mission' deactivated
FakeAutopilot: setting landed to 1
FakeAutopilot: sending mode completed (id=5)
Got matching ModeCompleted message, result: 0
Setting current mission item to 7
Mission completed
)V0G0N");
  log_capture->checkHasNoErrors();
}

TEST_F(MissionExecutionTester, persistence)
{
  // Test persistent state
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      px4_ros2::Waypoint({47.34820919, 8.54395214, 503}),
      px4_ros2::Waypoint({47.34720919, 8.54385214, 502}),
      px4_ros2::ActionItem("customActionInterruptible", {}),
      px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
      px4_ros2::ActionItem("rtl")});

  bool should_interrupt = true;
  int num_run_calls{0};
  const auto on_action_run = [&](const px4_ros2::ActionArguments & arguments)
    {
      if (should_interrupt) {
        // When the action is first activated, switch to position control, which deactivates the mission execution.
        fake_autopilot->setLanded(true);
        fake_autopilot->setModeAndArm(px4_ros2::ModeBase::kModeIDPosctl, 0);
        should_interrupt = false;
        EXPECT_FALSE(arguments.resuming());
        return false;
      }
      EXPECT_TRUE(arguments.resuming());
      return true;
    };
  {
    MissionExecutorTest mission_executor("my mission",
      px4_ros2::MissionExecutor::Configuration().withPersistenceFile(persistence_file)
      .addCustomAction<CustomActionInterruptible>(on_action_run, num_run_calls)
      .withTrajectoryExecutor<TrajectoryExecutorTest>(),
      *node, fake_autopilot);
    ASSERT_TRUE(mission_executor.doRegister());
    mission_executor.setMission(mission);

    bool deactivated{false};
    mission_executor.onDeactivated(
      [&] {
        deactivated = true;
      });

    mission_executor.setModeAndArm(mission_executor.modeId());

    // Wait for deactivation
    EXPECT_TRUE(waitFor(node, [&deactivated] {return deactivated;}));

    EXPECT_EQ(num_run_calls, 1);
  }

  // Simulate reboot. The mission should be continued
  {
    MissionExecutorTest mission_executor("my mission",
      px4_ros2::MissionExecutor::Configuration().withPersistenceFile(persistence_file)
      .addCustomAction<CustomActionInterruptible>(on_action_run, num_run_calls)
      .withTrajectoryExecutor<TrajectoryExecutorTest>(),
      *node, fake_autopilot);
    ASSERT_TRUE(mission_executor.doRegister());
    mission_executor.setMission(mission);

    bool mission_completed{false};
    mission_executor.onCompleted([&mission_completed] {mission_completed = true;});

    mission_executor.setModeAndArm(mission_executor.modeId());

    // Wait for the mission to complete
    EXPECT_TRUE(waitFor(node, [&mission_completed] {return mission_completed;}));

    EXPECT_EQ(num_run_calls, 2);
  }

  log_capture->expectEqual(
    R"V0G0N(
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running trajectory (start: 1, end: 2, stop: 1)
Running mode 100
FakeAutopilot: setting mode (id=100)
Mode 'my mission' activated
Waypoint[1]: 47.348209 8.543952 503.000, frame: 0
Setting current mission item to 2
Waypoint[2]: 47.347209 8.543852 502.000, frame: 0
Setting current mission item to 3
Running action 'customActionInterruptible'
FakeAutopilot: setting landed to 1
FakeAutopilot: setting mode (id=2)
Mode executor 'my mission' deactivated (1)
Saving persistent state to file '/tmp/mission_execution_test_persistence.tmp'
Mode 'my mission' deactivated
Unregistering
Restored mission state to index 3
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 3
Running action 'onResume'
Running action 'customActionInterruptible'
Setting current mission item to 4
Running trajectory (start: 4, end: 4, stop: 0)
Mode 'my mission' activated
Waypoint[4]: 47.398209 8.545952 500.000, frame: 0
Setting current mission item to 5
Running action 'rtl'
Running mode 5
FakeAutopilot: setting mode (id=5)
Mode 'my mission' deactivated
FakeAutopilot: setting landed to 1
FakeAutopilot: sending mode completed (id=5)
Got matching ModeCompleted message, result: 0
Setting current mission item to 6
Mission completed
Unregistering
)V0G0N");
  log_capture->checkHasNoErrors();
}

TEST_F(MissionExecutionTester, persistenceWithChange)
{
  // Test persistent state with a different mission
  bool should_interrupt = true;
  int num_run_calls{0};
  const auto on_action_run = [&](const px4_ros2::ActionArguments & arguments)
    {
      if (should_interrupt) {
        // When the action is first activated, switch to position control, which deactivates the mission execution.
        fake_autopilot->setLanded(true);
        fake_autopilot->setModeAndArm(px4_ros2::ModeBase::kModeIDPosctl, 0);
        should_interrupt = false;
        return false;
      }
      return true;
    };
  {
    const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
        px4_ros2::Waypoint({47.34820919, 8.54395214, 503}),
        px4_ros2::Waypoint({47.34720919, 8.54385214, 502}),
        px4_ros2::ActionItem("customActionInterruptible", {}),
        px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
        px4_ros2::ActionItem("rtl")});

    MissionExecutorTest mission_executor("my mission",
      px4_ros2::MissionExecutor::Configuration().withPersistenceFile(persistence_file)
      .addCustomAction<CustomActionInterruptible>(on_action_run, num_run_calls)
      .withTrajectoryExecutor<TrajectoryExecutorTest>(),
      *node, fake_autopilot);
    ASSERT_TRUE(mission_executor.doRegister());
    mission_executor.setMission(mission);

    bool deactivated{false};
    mission_executor.onDeactivated(
      [&] {
        deactivated = true;
      });

    mission_executor.setModeAndArm(mission_executor.modeId());

    // Wait for deactivation
    EXPECT_TRUE(waitFor(node, [&deactivated] {return deactivated;}));

    EXPECT_EQ(num_run_calls, 1);
  }

  // Simulate reboot. The stored mission state should be discarded as the mission is different
  {
    const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
        px4_ros2::Waypoint({47.34820919, 8.54395214, 503}),
        px4_ros2::ActionItem("customActionInterruptible", {}),
        px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
        px4_ros2::ActionItem("rtl")});

    MissionExecutorTest mission_executor("my mission",
      px4_ros2::MissionExecutor::Configuration().withPersistenceFile(persistence_file)
      .addCustomAction<CustomActionInterruptible>(on_action_run, num_run_calls)
      .withTrajectoryExecutor<TrajectoryExecutorTest>(),
      *node, fake_autopilot);
    ASSERT_TRUE(mission_executor.doRegister());
    mission_executor.setMission(mission);

    bool mission_completed{false};
    mission_executor.onCompleted([&mission_completed] {mission_completed = true;});

    mission_executor.setModeAndArm(mission_executor.modeId());

    // Wait for the mission to complete
    EXPECT_TRUE(waitFor(node, [&mission_completed] {return mission_completed;}));

    EXPECT_EQ(num_run_calls, 2);
  }

  log_capture->expectEqual(
    R"V0G0N(
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running trajectory (start: 1, end: 2, stop: 1)
Running mode 100
FakeAutopilot: setting mode (id=100)
Mode 'my mission' activated
Waypoint[1]: 47.348209 8.543952 503.000, frame: 0
Setting current mission item to 2
Waypoint[2]: 47.347209 8.543852 502.000, frame: 0
Setting current mission item to 3
Running action 'customActionInterruptible'
FakeAutopilot: setting landed to 1
FakeAutopilot: setting mode (id=2)
Mode executor 'my mission' deactivated (1)
Saving persistent state to file '/tmp/mission_execution_test_persistence.tmp'
Mode 'my mission' deactivated
Unregistering
Stored mission state checksum mismatch or index out of bounds, discarding
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running trajectory (start: 1, end: 1, stop: 1)
Running mode 100
FakeAutopilot: setting mode (id=100)
Mode 'my mission' activated
Waypoint[1]: 47.348209 8.543952 503.000, frame: 0
Setting current mission item to 2
Running action 'customActionInterruptible'
Setting current mission item to 3
Running trajectory (start: 3, end: 3, stop: 0)
Waypoint[3]: 47.398209 8.545952 500.000, frame: 0
Setting current mission item to 4
Running action 'rtl'
Running mode 5
FakeAutopilot: setting mode (id=5)
Mode 'my mission' deactivated
FakeAutopilot: setting landed to 1
FakeAutopilot: sending mode completed (id=5)
Got matching ModeCompleted message, result: 0
Setting current mission item to 5
Mission completed
Unregistering
)V0G0N");
  log_capture->checkHasNoErrors();
}

class CustomActionCameraTriggerTest : public px4_ros2::ActionInterface
{
public:
  explicit CustomActionCameraTriggerTest(px4_ros2::ModeBase & mode)
  : _node(mode.node()) {}
  std::string name() const override {return "customActionCameraTrigger";}

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    const auto action = arguments.at<std::string>("action");
    if (action == "start") {
      RCLCPP_INFO(_node.get_logger(), "Starting camera trigger");
      _state = handler->storeState(name(), arguments);
    } else if (action == "stop") {
      RCLCPP_INFO(_node.get_logger(), "Stopping camera trigger");
      _state.reset();
    } else {
      RCLCPP_ERROR(_node.get_logger(), "Unknown action '%s'", action.c_str());
    }
    on_completed();
  }
  bool shouldStopAtWaypoint(const px4_ros2::ActionArguments & arguments) override {return false;}

  void deactivate() override
  {
    RCLCPP_INFO(
      _node.get_logger(), "Stopping camera trigger (from deactivate), was active: %s",
      _state ? "yes" : "no");
    _state.reset();
  }

private:
  std::unique_ptr<px4_ros2::ActionStateKeeper> _state;
  rclcpp::Node & _node;
};

TEST_F(MissionExecutionTester, resumeWithActions)
{
  // Test changing trajectory settings, continuous actions and restoring them after resuming
  px4_ros2::MissionDefaults defaults{};
  defaults.trajectory_options.horizontal_velocity = 10.0;
  defaults.trajectory_options.vertical_velocity = 5.0;
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      px4_ros2::Waypoint({47.34820919, 8.54395214, 503}),
      px4_ros2::ActionItem(
        "changeSettings",
        px4_ros2::ActionArguments{{{"horizontalVelocity", 15.0}}}),
      px4_ros2::Waypoint({47.34820919, 8.54395214, 504}),
      px4_ros2::ActionItem(
        "customActionCameraTrigger",
        px4_ros2::ActionArguments{{{"action", "start"}}}),
      px4_ros2::ActionItem(
        "changeSettings",
        px4_ros2::ActionArguments{{{"horizontalVelocity", 18.0}}}),
      px4_ros2::ActionItem("changeSettings", px4_ros2::ActionArguments{{{"maxHeadingRate", 55.0}}}),
      px4_ros2::Waypoint({47.34720919, 8.54385214, 502}),
      px4_ros2::ActionItem("customActionInterruptible", {}),
      px4_ros2::ActionItem(
        "customActionCameraTrigger",
        px4_ros2::ActionArguments{{{"action", "stop"}}}),
      px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
      px4_ros2::ActionItem("changeSettings", px4_ros2::ActionArguments{{{"resetAll", true}}}),
      px4_ros2::Waypoint({47.39820919, 8.54595214, 501}),
      px4_ros2::ActionItem(
        "changeSettings",
        px4_ros2::ActionArguments{{{"verticalVelocity", 7.5}}}),
      px4_ros2::Waypoint({47.39820919, 8.54595214, 502}),
      px4_ros2::ActionItem("customActionInterruptible", {}),
      px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
      px4_ros2::ActionItem("rtl")},
    defaults);

  bool should_interrupt = true;
  int num_run_calls{0};
  const auto on_action_run = [&](const px4_ros2::ActionArguments & arguments)
    {
      if (should_interrupt) {
        // When the action is activated, switch to position control, which deactivates the mission execution.
        fake_autopilot->setLanded(true);
        fake_autopilot->setModeAndArm(px4_ros2::ModeBase::kModeIDPosctl, 0);
        should_interrupt = false;
        EXPECT_FALSE(arguments.resuming());
        return false;
      }
      EXPECT_TRUE(arguments.resuming());
      should_interrupt = true;
      return true;
    };
  auto init_mission_executor = [&]
    {
      const bool verbose_trajectory_output = true;
      auto mission_executor = std::make_shared<MissionExecutorTest>(
        "my mission",
        px4_ros2::MissionExecutor::Configuration().withPersistenceFile(persistence_file)
        .addCustomAction<CustomActionInterruptible>(on_action_run, num_run_calls)
        .addCustomAction<CustomActionCameraTriggerTest>()
        .withTrajectoryExecutor<TrajectoryExecutorTest>(verbose_trajectory_output),
        *node, fake_autopilot);
      EXPECT_TRUE(mission_executor->doRegister());
      mission_executor->setMission(mission);
      return mission_executor;
    };

  // Run the mission until the first interruption
  {
    auto mission_executor = init_mission_executor();

    bool deactivated{false};
    mission_executor->onDeactivated(
      [&] {
        deactivated = true;
      });

    mission_executor->setModeAndArm(mission_executor->modeId());

    // Wait for deactivation
    EXPECT_TRUE(waitFor(node, [&deactivated] {return deactivated;}));

    EXPECT_EQ(num_run_calls, 1);
  }

  // Simulate reboot. The mission should be continued, and actions restored
  {
    auto mission_executor = init_mission_executor();

    bool deactivated{false};
    mission_executor->onDeactivated(
      [&] {
        deactivated = true;
      });

    // Wait for land detected (this ensures the onResume action goes through the takeoff sequence)
    bool land_detected = false;
    fake_autopilot->setLanded(true);
    mission_executor->landDetected()->onUpdate(
      [&land_detected](const px4_msgs::msg::
      VehicleLandDetected & vehicle_land_detected) {land_detected = vehicle_land_detected.landed;});
    EXPECT_TRUE(waitFor(node, [&land_detected] {return land_detected;}));

    mission_executor->setModeAndArm(mission_executor->modeId());

    // Wait for deactivation
    EXPECT_TRUE(waitFor(node, [&deactivated] {return deactivated;}));

    EXPECT_EQ(num_run_calls, 3);
  }

  // Simulate reboot. The mission should be continued, and actions restored
  {
    auto mission_executor = init_mission_executor();

    // Wait for land detected (this ensures the onResume action goes through the takeoff sequence)
    bool land_detected = false;
    fake_autopilot->setLanded(true);
    mission_executor->landDetected()->onUpdate(
      [&land_detected](const px4_msgs::msg::
      VehicleLandDetected & vehicle_land_detected) {land_detected = vehicle_land_detected.landed;});
    EXPECT_TRUE(waitFor(node, [&land_detected] {return land_detected;}));

    bool mission_completed{false};
    mission_executor->onCompleted([&mission_completed] {mission_completed = true;});

    mission_executor->setModeAndArm(mission_executor->modeId());

    // Wait for the mission to complete
    EXPECT_TRUE(waitFor(node, [&mission_completed] {return mission_completed;}));

    EXPECT_EQ(num_run_calls, 4);
  }

  log_capture->expectEqual(
    R"V0G0N(
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running trajectory (start: 1, end: 1, stop: 0)
Running mode 100
FakeAutopilot: setting mode (id=100)
Mode 'my mission' activated
Waypoint[1]: 47.348209 8.543952 503.000, frame: 0
       With: hor v=10.0 m/s, vert v=5.0 m/s, yaw rate=nan deg/s
Setting current mission item to 2
Running action 'changeSettings'
Adding continuous action id 0, action name 'changeSettings'
Setting current mission item to 3
Running trajectory (start: 3, end: 3, stop: 0)
Waypoint[3]: 47.348209 8.543952 504.000, frame: 0
       With: hor v=15.0 m/s, vert v=5.0 m/s, yaw rate=nan deg/s
Setting current mission item to 4
Running action 'customActionCameraTrigger'
Starting camera trigger
Adding continuous action id 1, action name 'customActionCameraTrigger'
Setting current mission item to 5
Running action 'changeSettings'
Adding continuous action id 2, action name 'changeSettings'
Removing continuous action id 0
Setting current mission item to 6
Running action 'changeSettings'
Adding continuous action id 3, action name 'changeSettings'
Removing continuous action id 2
Setting current mission item to 7
Running trajectory (start: 7, end: 7, stop: 1)
Waypoint[7]: 47.347209 8.543852 502.000, frame: 0
       With: hor v=18.0 m/s, vert v=5.0 m/s, yaw rate=55.0 deg/s
Setting current mission item to 8
Running action 'customActionInterruptible'
FakeAutopilot: setting landed to 1
FakeAutopilot: setting mode (id=2)
Mode executor 'my mission' deactivated (1)
Saving persistent state to file '/tmp/mission_execution_test_persistence.tmp'
Stopping camera trigger (from deactivate), was active: yes
Mode 'my mission' deactivated
Unregistering
Restored mission state to index 8
FakeAutopilot: setting landed to 1
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 8
Running action 'onResume'
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Resume: navigating to waypoint before the current one
Running custom trajectory (1 items)
Running trajectory (start: 0, end: 0, stop: 0)
Running mode 100
FakeAutopilot: setting mode (id=100)
Mode 'my mission' activated
Waypoint[0]: 47.347209 8.543852 502.000, frame: 0
       With: hor v=10.0 m/s, vert v=5.0 m/s, yaw rate=nan deg/s
Starting camera trigger
Adding continuous action id 0, action name 'customActionCameraTrigger'
Adding continuous action id 1, action name 'changeSettings'
Restored 2 actions (new stored count: 2)
Running action 'customActionInterruptible'
Setting current mission item to 9
Running action 'customActionCameraTrigger'
Stopping camera trigger
Removing continuous action id 0
Setting current mission item to 10
Running trajectory (start: 10, end: 10, stop: 0)
Waypoint[10]: 47.398209 8.545952 500.000, frame: 0
       With: hor v=18.0 m/s, vert v=5.0 m/s, yaw rate=55.0 deg/s
Setting current mission item to 11
Running action 'changeSettings'
Removing continuous action id 1
Setting current mission item to 12
Running trajectory (start: 12, end: 12, stop: 0)
Waypoint[12]: 47.398209 8.545952 501.000, frame: 0
       With: hor v=10.0 m/s, vert v=5.0 m/s, yaw rate=nan deg/s
Setting current mission item to 13
Running action 'changeSettings'
Adding continuous action id 2, action name 'changeSettings'
Setting current mission item to 14
Running trajectory (start: 14, end: 14, stop: 1)
Waypoint[14]: 47.398209 8.545952 502.000, frame: 0
       With: hor v=10.0 m/s, vert v=7.5 m/s, yaw rate=nan deg/s
Setting current mission item to 15
Running action 'customActionInterruptible'
FakeAutopilot: setting landed to 1
FakeAutopilot: setting mode (id=2)
Mode executor 'my mission' deactivated (1)
Saving persistent state to file '/tmp/mission_execution_test_persistence.tmp'
Stopping camera trigger (from deactivate), was active: no
Mode 'my mission' deactivated
Unregistering
Restored mission state to index 15
FakeAutopilot: setting landed to 1
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 15
Running action 'onResume'
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Resume: navigating to waypoint before the current one
Running custom trajectory (1 items)
Running trajectory (start: 0, end: 0, stop: 0)
Running mode 100
FakeAutopilot: setting mode (id=100)
Mode 'my mission' activated
Waypoint[0]: 47.398209 8.545952 502.000, frame: 0
       With: hor v=10.0 m/s, vert v=5.0 m/s, yaw rate=nan deg/s
Adding continuous action id 0, action name 'changeSettings'
Restored 1 actions (new stored count: 1)
Running action 'customActionInterruptible'
Setting current mission item to 16
Running trajectory (start: 16, end: 16, stop: 0)
Waypoint[16]: 47.398209 8.545952 500.000, frame: 0
       With: hor v=10.0 m/s, vert v=7.5 m/s, yaw rate=nan deg/s
Setting current mission item to 17
Running action 'rtl'
Running mode 5
FakeAutopilot: setting mode (id=5)
Mode 'my mission' deactivated
FakeAutopilot: setting landed to 1
FakeAutopilot: sending mode completed (id=5)
Got matching ModeCompleted message, result: 0
Setting current mission item to 18
Mission completed
Unregistering
Removing continuous action id 0
)V0G0N");
  log_capture->checkHasNoErrors();
}


class OnResumeWithState : public px4_ros2::ActionInterface
{
public:
  explicit OnResumeWithState(px4_ros2::ModeBase & mode)
  : _node(mode.node())
  {
  }

  ~OnResumeWithState() override = default;
  std::string name() const override {return "onResume";}

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    const auto action = arguments.at<std::string>("action");
    if (action == "storeState") {
      nlohmann::json state;
      state["custom"] = "value";
      state["a"] = true;
      _state = handler->storeState(name(), px4_ros2::ActionArguments(state));
    } else if (action == "resume") {
      RCLCPP_INFO(_node.get_logger(), "Resuming");
      EXPECT_EQ(arguments.at<std::string>("custom"), "value");
      EXPECT_EQ(arguments.at<bool>("a"), true);
    } else {
      RCLCPP_ERROR(_node.get_logger(), "Unknown action '%s'", action.c_str());
    }
    on_completed();
  }

private:
  rclcpp::Node & _node;
  std::unique_ptr<px4_ros2::ActionStateKeeper> _state;
};

TEST_F(MissionExecutionTester, resumeWithState)
{
  // Test a custom onResume action that stores and restores a custom state
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      px4_ros2::Waypoint({47.34820919, 8.54395214, 503}),
      px4_ros2::Waypoint({47.34720919, 8.54385214, 502}),
      px4_ros2::ActionItem("customActionInterruptible", {}),
      px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
      px4_ros2::ActionItem("rtl")});

  bool should_interrupt = true;
  int num_run_calls{0};
  const auto on_action_run = [&](const px4_ros2::ActionArguments & arguments)
    {
      if (should_interrupt) {
        // When the action is first activated, switch to position control, which deactivates the mission execution.
        fake_autopilot->setLanded(true);
        fake_autopilot->setModeAndArm(px4_ros2::ModeBase::kModeIDPosctl, 0);
        should_interrupt = false;
        return false;
      }
      return true;
    };
  {
    MissionExecutorTest mission_executor("my mission",
      px4_ros2::MissionExecutor::Configuration().withPersistenceFile(persistence_file)
      .addCustomAction<CustomActionInterruptible>(on_action_run, num_run_calls)
      .addCustomAction<OnResumeWithState>()
      .withTrajectoryExecutor<TrajectoryExecutorTest>(),
      *node, fake_autopilot);
    ASSERT_TRUE(mission_executor.doRegister());
    mission_executor.setMission(mission);

    bool deactivated{false};
    mission_executor.onDeactivated(
      [&] {
        deactivated = true;
      });

    mission_executor.setModeAndArm(mission_executor.modeId());

    // Wait for deactivation
    EXPECT_TRUE(waitFor(node, [&deactivated] {return deactivated;}));
  }

  // Simulate reboot. The mission should be continued, and the onResume state restored
  {
    MissionExecutorTest mission_executor("my mission",
      px4_ros2::MissionExecutor::Configuration().withPersistenceFile(persistence_file)
      .addCustomAction<CustomActionInterruptible>(on_action_run, num_run_calls)
      .addCustomAction<OnResumeWithState>()
      .withTrajectoryExecutor<TrajectoryExecutorTest>(),
      *node, fake_autopilot);
    ASSERT_TRUE(mission_executor.doRegister());
    mission_executor.setMission(mission);

    bool mission_completed{false};
    mission_executor.onCompleted([&mission_completed] {mission_completed = true;});

    mission_executor.setModeAndArm(mission_executor.modeId());

    // Wait for the mission to complete
    EXPECT_TRUE(waitFor(node, [&mission_completed] {return mission_completed;}));
  }

  log_capture->expectEqual(
    R"V0G0N(
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running trajectory (start: 1, end: 2, stop: 1)
Running mode 100
FakeAutopilot: setting mode (id=100)
Mode 'my mission' activated
Waypoint[1]: 47.348209 8.543952 503.000, frame: 0
Setting current mission item to 2
Waypoint[2]: 47.347209 8.543852 502.000, frame: 0
Setting current mission item to 3
Running action 'customActionInterruptible'
FakeAutopilot: setting landed to 1
FakeAutopilot: setting mode (id=2)
Mode executor 'my mission' deactivated (1)
Adding continuous action id 0, action name 'onResume'
Saving persistent state to file '/tmp/mission_execution_test_persistence.tmp'
Mode 'my mission' deactivated
Unregistering
Restored mission state to index 3
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 3
Running action 'onResume'
Resuming
Running action 'customActionInterruptible'
Setting current mission item to 4
Running trajectory (start: 4, end: 4, stop: 0)
Mode 'my mission' activated
Waypoint[4]: 47.398209 8.545952 500.000, frame: 0
Setting current mission item to 5
Running action 'rtl'
Running mode 5
FakeAutopilot: setting mode (id=5)
Mode 'my mission' deactivated
FakeAutopilot: setting landed to 1
FakeAutopilot: sending mode completed (id=5)
Got matching ModeCompleted message, result: 0
Setting current mission item to 6
Mission completed
Unregistering
)V0G0N");
  log_capture->checkHasNoErrors();
}

class CustomActionTriggerFailure : public px4_ros2::ActionInterface
{
public:
  explicit CustomActionTriggerFailure(px4_ros2::ModeBase & mode) {}
  std::string name() const override {return "customActionTriggerFailure";}

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    _handler = handler;
    handler->runAction(
      "nonExistentAction", {}, []
      {
        EXPECT_TRUE(false) << "on_completed called, but should never be";
      });

    // Call on_completed, which must not do anything, since the action failed and abort was called
    on_completed();
  }

  std::shared_ptr<px4_ros2::ActionHandler> handler() {return _handler;}

private:
  std::shared_ptr<px4_ros2::ActionHandler> _handler;
};

TEST_F(MissionExecutionTester, failureDefault)
{
  // Test that the default failure action is triggered by trying to run an unknown action.
  // The failure action is expected to run RTL first (which fails), then fall back to Descend (which succeeds)
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      px4_ros2::ActionItem("customActionTriggerFailure")});

  MissionExecutorTest mission_executor("my mission",
    px4_ros2::MissionExecutor::Configuration().addCustomAction<CustomActionTriggerFailure>(), *node,
    fake_autopilot);
  ASSERT_TRUE(mission_executor.doRegister());
  mission_executor.setMission(mission);

  mission_executor.mode_change_request = [&](int nav_state)
    {
      if (nav_state == px4_ros2::ModeBase::kModeIDRtl) {
        RCLCPP_INFO(node->get_logger(), "Rejecting mode %i", nav_state);
        return false;
      }
      mission_executor.setModeAndArm(nav_state, true);
      return true;
    };

  mission_executor.setModeAndArm(mission_executor.modeId());

  // Mission will not complete, just check the log output
  log_capture->expectEqual(
    R"V0G0N(
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running action 'customActionTriggerFailure'
Running action 'nonExistentAction'
Trying to run nonexistent action 'nonExistentAction'
Aborting mission (reason: actionDoesNotExist, recursion level: 0)
Running action 'onFailure'
Running mode 5
Rejecting mode 5
Running mode 12
FakeAutopilot: setting mode (id=12)
FakeAutopilot: sending mode completed (id=12)
Got matching ModeCompleted message, result: 0
onFailure action completed
)V0G0N");
}

TEST_F(MissionExecutionTester, failureInvalidHandler)
{
  // Test that the action handler becomes invalid after a failure/abort.
  // It ensures that actions using timers to run actions after being activated are canceled when interrupted
  // (either by an abort or deactivation).
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      px4_ros2::ActionItem("customActionTriggerFailure")});

  px4_ros2::MissionExecutor::Configuration config;
  std::shared_ptr<CustomActionTriggerFailure> action;
  config.custom_actions_factory.emplace_back(
    [&action](px4_ros2::ModeBase & mode)
    {
      action = std::make_shared<CustomActionTriggerFailure>(mode);
      return action;
    });
  MissionExecutorTest mission_executor("my mission", config, *node, fake_autopilot);
  ASSERT_TRUE(mission_executor.doRegister());
  mission_executor.setMission(mission);

  mission_executor.setModeAndArm(mission_executor.modeId());

  ASSERT_TRUE(
    waitFor(
      node, [&action]
      {
        return action->handler() != nullptr;
      }));

  EXPECT_FALSE(action->handler()->isValid());
}

class CustomOnFailureAction : public px4_ros2::ActionInterface
{
public:
  explicit CustomOnFailureAction(px4_ros2::ModeBase & mode)
  : _node(mode.node())
  {}

  std::string name() const override {return "onFailure";}

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    const int recursion_level = arguments.at<int>("recursionLevel");
    const std::string reason = arguments.at<std::string>("reason");
    int index = -1;
    if (arguments.contains("currentIndex")) {
      index = arguments.at<int>("currentIndex");
    }
    RCLCPP_INFO(
      _node.get_logger(), "Failure handling: recursion level: %i, reason: %s, current index: %i", recursion_level,
      reason.c_str(), index);
    handler->runAction(
      "nonExistentAction", {}, []
      {
        EXPECT_TRUE(false) << "on_completed called, but should never be";
      });
  }

private:
  rclcpp::Node & _node;
};

TEST_F(MissionExecutionTester, failureCustom)
{
  // Test the recursion limit with a custom failure action
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      px4_ros2::ActionItem("customActionTriggerFailure")});

  MissionExecutorTest mission_executor("my mission",
    px4_ros2::MissionExecutor::Configuration().addCustomAction<CustomActionTriggerFailure>().
    addCustomAction<CustomOnFailureAction>(), *node, fake_autopilot);
  ASSERT_TRUE(mission_executor.doRegister());
  mission_executor.setMission(mission);

  mission_executor.setModeAndArm(mission_executor.modeId());

  // Mission will not complete, just check the log output
  log_capture->expectEqual(
    R"V0G0N(
FakeAutopilot: setting mode (id=100)
Mode executor 'my mission' activated
Setting current mission item to 0
Running action 'takeoff'
Running takeoff mode with altitude: nan, heading: nan
FakeAutopilot: setting mode (id=17)
FakeAutopilot: setting landed to 0
FakeAutopilot: sending mode completed (id=17)
Got matching ModeCompleted message, result: 0
Setting current mission item to 1
Running action 'customActionTriggerFailure'
Running action 'nonExistentAction'
Trying to run nonexistent action 'nonExistentAction'
Aborting mission (reason: actionDoesNotExist, recursion level: 0)
Running action 'onFailure'
Failure handling: recursion level: 1, reason: actionDoesNotExist, current index: 1
Running action 'nonExistentAction'
Trying to run nonexistent action 'nonExistentAction'
Aborting mission (reason: actionDoesNotExist, recursion level: 1)
Running action 'onFailure'
Failure handling: recursion level: 2, reason: actionDoesNotExist, current index: 1
Running action 'nonExistentAction'
Trying to run nonexistent action 'nonExistentAction'
Aborting mission (reason: actionDoesNotExist, recursion level: 2)
Running action 'onFailure'
Failure handling: recursion level: 3, reason: actionDoesNotExist, current index: 1
Running action 'nonExistentAction'
Trying to run nonexistent action 'nonExistentAction'
Aborting mission (reason: actionDoesNotExist, recursion level: 3)
Running action 'onFailure'
Failure handling: recursion level: 4, reason: actionDoesNotExist, current index: 1
Running action 'nonExistentAction'
Trying to run nonexistent action 'nonExistentAction'
Aborting mission (reason: actionDoesNotExist, recursion level: 4)
Running action 'onFailure'
Failure handling: recursion level: 5, reason: actionDoesNotExist, current index: 1
Running action 'nonExistentAction'
Trying to run nonexistent action 'nonExistentAction'
Aborting mission (reason: actionDoesNotExist, recursion level: 5)
Running action 'onFailure'
Failure handling: recursion level: 6, reason: actionDoesNotExist, current index: 1
Running action 'nonExistentAction'
Trying to run nonexistent action 'nonExistentAction'
Aborting mission (reason: actionDoesNotExist, recursion level: 6)
Running action 'onFailure'
Failure handling: recursion level: 7, reason: actionDoesNotExist, current index: 1
Running action 'nonExistentAction'
Trying to run nonexistent action 'nonExistentAction'
Aborting mission (reason: actionDoesNotExist, recursion level: 7)
Running action 'onFailure'
Failure handling: recursion level: 8, reason: actionDoesNotExist, current index: 1
Running action 'nonExistentAction'
Trying to run nonexistent action 'nonExistentAction'
Aborting mission (reason: actionDoesNotExist, recursion level: 8)
Running action 'onFailure'
Failure handling: recursion level: 9, reason: actionDoesNotExist, current index: 1
Running action 'nonExistentAction'
Trying to run nonexistent action 'nonExistentAction'
Aborting mission (reason: actionDoesNotExist, recursion level: 9)
Running action 'onFailure'
Failure handling: recursion level: 10, reason: actionDoesNotExist, current index: 1
Running action 'nonExistentAction'
Trying to run nonexistent action 'nonExistentAction'
Aborting mission (reason: actionDoesNotExist, recursion level: 10)
Running action 'onFailure'
Failure handling: recursion level: 11, reason: actionDoesNotExist, current index: 1
Running action 'nonExistentAction'
Trying to run nonexistent action 'nonExistentAction'
Aborting mission (reason: actionDoesNotExist, recursion level: 11)
Maximum abort recursion level reached
)V0G0N");
}

class CustomActionReadyness : public px4_ros2::ActionInterface
{
public:
  CustomActionReadyness(px4_ros2::ModeBase & mode, std::function<bool()> on_can_run)
  : _on_can_run(std::move(on_can_run)) {}
  std::string name() const override {return "customActionReadyness";}

  bool canRun(
    const px4_ros2::ActionArguments & arguments,
    std::vector<std::string> & errors) override
  {
    return _on_can_run();
  }

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    on_completed();
  }

private:
  std::function<bool()> _on_can_run;
};

TEST_F(MissionExecutionTester, readyness)
{
  // Test onReadynessUpdate callback
  const px4_ros2::Mission mission({px4_ros2::ActionItem("takeoff"),
      px4_ros2::ActionItem("customActionReadyness", {}),
      px4_ros2::ActionItem("unknownAction", {}),
      px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
      px4_ros2::ActionItem("rtl")});

  bool action_can_run = false;
  const auto on_can_run = [&]
    {
      return action_can_run;
    };
  MissionExecutorTest mission_executor("my mission",
    px4_ros2::MissionExecutor::Configuration()
    .addCustomAction<CustomActionReadyness>(on_can_run)
    .withTrajectoryExecutor<TrajectoryExecutorTest>(),
    *node, fake_autopilot);
  ASSERT_TRUE(mission_executor.doRegister());

  int num_readyness_updates{0};
  std::vector<std::string> last_errors;
  mission_executor.onReadynessUpdate(
    [&](bool ready, const std::vector<std::string> & errors)
    {
      ++num_readyness_updates;
      last_errors = errors;
    });

  // Ensure we get a report even before setting a mission
  {
    ASSERT_TRUE(waitFor(node, [&num_readyness_updates] {return num_readyness_updates == 1;}));
    EXPECT_EQ(last_errors, std::vector<std::string>{{std::string("No mission")}});
  }

  mission_executor.setMission(mission);

  // The mission contains some errors
  {
    ASSERT_TRUE(waitFor(node, [&num_readyness_updates] {return num_readyness_updates == 2;}));
    const auto expected = std::vector<std::string>{{std::string(
        "Mission contains unknown action 'unknownAction'"),
      "Action 'customActionReadyness' is not ready yet"}};
    EXPECT_EQ(last_errors, expected);
  }

  // Change action readyness
  action_can_run = true;
  {
    ASSERT_TRUE(waitFor(node, [&num_readyness_updates] {return num_readyness_updates == 3;}));
    const auto expected = std::vector<std::string>{{std::string(
        "Mission contains unknown action 'unknownAction'")}};
    EXPECT_EQ(last_errors, expected);
  }

  // Change the mission
  const px4_ros2::Mission mission_updated({px4_ros2::ActionItem("takeoff"),
      px4_ros2::ActionItem("customActionReadyness", {}),
      px4_ros2::Waypoint({47.39820919, 8.54595214, 500}),
      px4_ros2::ActionItem("rtl")});
  mission_executor.setMission(mission_updated);
  {
    ASSERT_TRUE(waitFor(node, [&num_readyness_updates] {return num_readyness_updates == 4;}));
    const auto expected = std::vector<std::string>{};
    EXPECT_EQ(last_errors, expected);
  }

  // Reset the mission
  mission_executor.resetMission();
  {
    ASSERT_TRUE(waitFor(node, [&num_readyness_updates] {return num_readyness_updates == 5;}));
    const auto expected = std::vector<std::string>{{"No mission"}};
    EXPECT_EQ(last_errors, expected);
  }
}
