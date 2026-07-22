/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_ros2/utils/vehicle_command_sender.hpp>
#include <px4_ros2_sdk/result.hpp>
#include <px4_ros2_sdk_interfaces/action/takeoff.hpp>
#include <px4_ros2_sdk_interfaces/srv/set_armed.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>

namespace px4_ros2::sdk {
/** \ingroup sdk
 *  @{
 */

/**
 * @brief Managed lifecycle node that exposes PX4 flight capabilities as a ROS 2
 *        service + action graph, backed by the px4_ros2_cpp device-driver core.
 *
 * This is the P0 slice of the flight-SDK RFC (doc/API_EXPANSION_RFC.md): the
 * bringup and the non-streaming "safe core". on_configure runs the L1 discovery
 * (waitForFMU) and fails closed if the FMU is absent; on_activate starts the
 * arm service and the takeoff action server. Arm is a one-shot service; takeoff
 * is a long-running action with feedback and cancellation.
 *
 * The maneuver here is driven directly through VehicleCommandSender (the
 * acknowledgement is the result). The RFC's full design routes takeoff/land/rtl
 * through ModeExecutorBase so completion rides mode_completed; that is deferred
 * to a later slice and noted in the RFC.
 *
 * px4_ros2_cpp's helpers take an rclcpp::Node&, so the server owns a plain L1
 * node for the device-driver interactions while itself remaining a LifecycleNode
 * (the ROS 2 graph surface).
 */
class FlightServer : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using SetArmed = px4_ros2_sdk_interfaces::srv::SetArmed;
  using Takeoff = px4_ros2_sdk_interfaces::action::Takeoff;
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;

  explicit FlightServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  // Command builders. Unused params are NaN, matching px4_ros2::ModeExecutorBase:
  // PX4 then reads only the params a command defines, and a value-initialized
  // (finite zero) field is never mistaken for a real setpoint. Static and pure
  // so the emitted command is unit-testable without an FMU.
  static px4_msgs::msg::VehicleCommand makeArmCommand(bool arm, bool force);

  /// param5/param6 (lat/lon) are NaN so Navigator targets the current position;
  /// a finite value there commands that literal coordinate (see NAV_TAKEOFF).
  static px4_msgs::msg::VehicleCommand makeTakeoffCommand(float altitude_amsl_m,
                                                          float heading = NAN);

 private:
  void handleArm(const std::shared_ptr<SetArmed::Request>& request,
                 const std::shared_ptr<SetArmed::Response>& response);

  rclcpp_action::GoalResponse handleTakeoffGoal(const rclcpp_action::GoalUUID& uuid,
                                                const std::shared_ptr<const Takeoff::Goal>& goal);
  rclcpp_action::CancelResponse handleTakeoffCancel(
      const std::shared_ptr<GoalHandleTakeoff>& handle);
  void handleTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff>& handle);
  void executeTakeoff(const std::shared_ptr<GoalHandleTakeoff>& handle);

  Result sendVehicleCommand(const px4_msgs::msg::VehicleCommand& command);

  std::string _topic_namespace_prefix;
  double _discovery_timeout_s{5.0};
  double _heartbeat_timeout_s{5.0};

  rclcpp::Node::SharedPtr _l1_node;
  std::unique_ptr<px4_ros2::VehicleCommandSender> _command_sender;

  rclcpp::Service<SetArmed>::SharedPtr _arm_service;
  rclcpp_action::Server<Takeoff>::SharedPtr _takeoff_action;
};

/** @}*/
}  // namespace px4_ros2::sdk
