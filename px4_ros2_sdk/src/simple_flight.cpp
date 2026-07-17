/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2_sdk/simple_flight.hpp>
#include <string>
#include <utility>

namespace px4_ros2::sdk {

SimpleFlight::SimpleFlight(rclcpp::Node::SharedPtr node, const std::string& server_namespace)
    : _node(std::move(node))
{
  const std::string base = server_namespace + "/flight_server";
  _arm_client = _node->create_client<SetArmed>(base + "/arm");
  _takeoff_client = rclcpp_action::create_client<Takeoff>(_node, base + "/takeoff");
}

Result SimpleFlight::setArmed(bool arm, bool force, std::chrono::milliseconds timeout)
{
  if (!_arm_client->wait_for_service(timeout)) {
    return Result::NotConnected;
  }
  auto request = std::make_shared<SetArmed::Request>();
  request->arm = arm;
  request->force = force;
  auto future = _arm_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(_node, future, timeout) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return Result::Timeout;
  }
  return future.get()->accepted ? Result::Success : Result::CommandDenied;
}

Result SimpleFlight::arm(bool force, std::chrono::milliseconds timeout)
{
  return setArmed(true, force, timeout);
}

Result SimpleFlight::disarm(std::chrono::milliseconds timeout)
{
  return setArmed(false, false, timeout);
}

Result SimpleFlight::takeoff(float altitude_amsl_m, std::chrono::milliseconds timeout)
{
  if (!_takeoff_client->wait_for_action_server(std::chrono::seconds(5))) {
    return Result::NotConnected;
  }
  Takeoff::Goal goal;
  goal.altitude_amsl_m = altitude_amsl_m;

  auto goal_future = _takeoff_client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(_node, goal_future, timeout) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return Result::Timeout;
  }
  const auto& goal_handle = goal_future.get();
  if (!goal_handle) {
    return Result::CommandDenied;  // goal rejected
  }

  auto result_future = _takeoff_client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(_node, result_future, timeout) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return Result::Timeout;
  }
  const auto& wrapped = result_future.get();
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result->success) {
    return Result::Success;
  }
  return Result::CommandDenied;
}

}  // namespace px4_ros2::sdk
