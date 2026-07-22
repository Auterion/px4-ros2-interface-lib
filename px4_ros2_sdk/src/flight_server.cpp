/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <cmath>
#include <memory>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <px4_ros2_sdk/flight_server.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <thread>

namespace px4_ros2::sdk {

using VehicleCommand = px4_msgs::msg::VehicleCommand;
using CallbackReturn = FlightServer::CallbackReturn;

FlightServer::FlightServer(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("flight_server", options)
{
  _topic_namespace_prefix = declare_parameter<std::string>("topic_namespace_prefix", "");
  _discovery_timeout_s = declare_parameter<double>("discovery_timeout_s", 5.0);
  _heartbeat_timeout_s = declare_parameter<double>("heartbeat_timeout_s", 5.0);
}

CallbackReturn FlightServer::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
  // px4_ros2_cpp helpers operate on an rclcpp::Node; own one for the L1 path.
  _l1_node = std::make_shared<rclcpp::Node>("flight_server_l1");
  _command_sender =
      std::make_unique<px4_ros2::VehicleCommandSender>(*_l1_node, _topic_namespace_prefix);

  // Blocking discovery is allowed here: no setpoint-update timer runs yet. The
  // full server also gates on messageCompatibilityCheck over the enabled
  // capability topics (see the RFC); the safe core here needs only the FMU.
  if (!px4_ros2::waitForFMU(*_l1_node, rclcpp::Duration::from_seconds(_discovery_timeout_s),
                            rclcpp::Duration::from_seconds(_heartbeat_timeout_s),
                            _topic_namespace_prefix)) {
    RCLCPP_WARN(get_logger(), "FMU not discovered; staying unconfigured");
    _command_sender.reset();
    _l1_node.reset();
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn FlightServer::on_activate(const rclcpp_lifecycle::State& /*state*/)
{
  _arm_service = create_service<SetArmed>(
      "~/arm", [this](const std::shared_ptr<SetArmed::Request>& req,
                      const std::shared_ptr<SetArmed::Response>& resp) { handleArm(req, resp); });

  _takeoff_action = rclcpp_action::create_server<Takeoff>(
      this, "~/takeoff",
      [this](const rclcpp_action::GoalUUID& uuid,
             const std::shared_ptr<const Takeoff::Goal>& goal) {
        return handleTakeoffGoal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandleTakeoff>& handle) {
        return handleTakeoffCancel(handle);
      },
      [this](const std::shared_ptr<GoalHandleTakeoff>& handle) { handleTakeoffAccepted(handle); });

  RCLCPP_INFO(get_logger(), "FlightServer active: ~/arm service and ~/takeoff action started");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FlightServer::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
  _arm_service.reset();
  _takeoff_action.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn FlightServer::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
  _arm_service.reset();
  _takeoff_action.reset();
  _command_sender.reset();
  _l1_node.reset();
  return CallbackReturn::SUCCESS;
}

VehicleCommand FlightServer::makeArmCommand(bool arm, bool force)
{
  VehicleCommand cmd{};
  cmd.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  cmd.param1 = arm ? 1.f : 0.f;
  cmd.param2 = force ? 21196.f : NAN;  // force magic value, else run preflight checks
  cmd.param3 = NAN;
  cmd.param4 = NAN;
  cmd.param5 = NAN;
  cmd.param6 = NAN;
  cmd.param7 = NAN;
  cmd.target_system = 0;
  cmd.target_component = 1;
  return cmd;
}

VehicleCommand FlightServer::makeTakeoffCommand(float altitude_amsl_m, float heading)
{
  VehicleCommand cmd{};
  cmd.command = VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
  cmd.param1 = NAN;
  cmd.param2 = NAN;
  cmd.param3 = NAN;
  cmd.param4 = heading;
  cmd.param5 = NAN;  // latitude
  cmd.param6 = NAN;  // longitude
  cmd.param7 = altitude_amsl_m;
  cmd.target_system = 0;
  cmd.target_component = 1;
  return cmd;
}

Result FlightServer::sendVehicleCommand(const VehicleCommand& command)
{
  if (!_command_sender) {
    return Result::NotConnected;
  }
  return fromCoreResult(_command_sender->sendCommandSync(command));
}

void FlightServer::handleArm(const std::shared_ptr<SetArmed::Request>& request,
                             const std::shared_ptr<SetArmed::Response>& response)
{
  const Result result = sendVehicleCommand(makeArmCommand(request->arm, request->force));
  response->accepted = result == Result::Success;
  response->message = resultToString(result);
}

rclcpp_action::GoalResponse FlightServer::handleTakeoffGoal(
    const rclcpp_action::GoalUUID& /*uuid*/, const std::shared_ptr<const Takeoff::Goal>& /*goal*/)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FlightServer::handleTakeoffCancel(
    const std::shared_ptr<GoalHandleTakeoff>& /*handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FlightServer::handleTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff>& handle)
{
  // Run the blocking command off the executor thread.
  std::thread{[this, handle]() { executeTakeoff(handle); }}.detach();
}

void FlightServer::executeTakeoff(const std::shared_ptr<GoalHandleTakeoff>& handle)
{
  const auto goal = handle->get_goal();
  auto feedback = std::make_shared<Takeoff::Feedback>();
  feedback->current_altitude_m = NAN;
  handle->publish_feedback(feedback);

  const Result result = sendVehicleCommand(makeTakeoffCommand(goal->altitude_amsl_m));

  auto action_result = std::make_shared<Takeoff::Result>();
  action_result->success = result == Result::Success;
  action_result->message = resultToString(result);
  if (result == Result::Success) {
    handle->succeed(action_result);
  } else {
    handle->abort(action_result);
  }
}

}  // namespace px4_ros2::sdk

RCLCPP_COMPONENTS_REGISTER_NODE(px4_ros2::sdk::FlightServer)
