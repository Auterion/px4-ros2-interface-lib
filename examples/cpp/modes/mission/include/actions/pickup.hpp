/****************************************************************************
* Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/vehicle_state/land_detected.hpp>
#include <px4_ros2/third_party/nlohmann/json.hpp>

class PickupAction : public px4_ros2::ActionInterface
{
public:
  explicit PickupAction(px4_ros2::ModeBase & mode)
  : _node(mode.node())
  {
    _land_detected = std::make_shared<px4_ros2::LandDetected>(mode);
  }
  std::string name() const override {return "pickup";}

  bool supportsResumeFromLanded() override {return true;}

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    // If currently landed, we proceed with the takeoff.
    // If that is interrupted, we don't want to land again, so we keep track of this state.
    // If landing gets interrupted, we still want to land again when resuming.
    const bool landed = _land_detected->lastValid(3s) && _land_detected->landed();
    const bool resuming_from_landed = arguments.resuming() && _continuing_from_landed;

    if (landed || resuming_from_landed) {
      _continuing_from_landed = true;

      const auto continue_after_takeoff = [handler, on_completed] {
          handler->controlAutoSetHome(true);
          handler->runAction("hold", px4_ros2::ActionArguments{{{"delay", 5}}}, on_completed);
        };

      if (resuming_from_landed) {
        RCLCPP_INFO(_node.get_logger(), "Resuming pickup action");
        continue_after_takeoff();
      } else {
        RCLCPP_INFO(_node.get_logger(), "Running pickup action from landed");
        const float altitude =
          arguments.contains("altitude") ? arguments.at<float>("altitude") : NAN;
        handler->runModeTakeoff(altitude, NAN, continue_after_takeoff);
      }

    } else {
      _continuing_from_landed = false;
      RCLCPP_INFO(_node.get_logger(), "Running pickup action (in-air)");

      // Go to Hold for a few seconds, then land
      handler->runAction(
        "hold", px4_ros2::ActionArguments{{{"delay", 5}}}, [handler]
        {
          // Prevent PX4 from updating the home position on the next takeoff
          handler->controlAutoSetHome(false);
          // The vehicle disarms after landing, and we don't want to continue with the mission immediately
          handler->runMode(px4_ros2::ModeBase::kModeIDLand, [] {});
        });
    }
  }

private:
  rclcpp::Node & _node;
  std::shared_ptr<px4_ros2::LandDetected> _land_detected;
  bool _continuing_from_landed{false};
};
