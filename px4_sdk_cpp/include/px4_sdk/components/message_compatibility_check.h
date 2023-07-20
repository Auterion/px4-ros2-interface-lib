/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>
using namespace std::chrono_literals;

// Set of all messages used by the SDK (<topic_name>[, <topic_type>])
#define ALL_PX4_SDK_MESSAGES \
	{"/fmu/in/arming_check_reply"},\
	{"/fmu/in/config_control_setpoints", "VehicleControlMode"},\
	{"/fmu/in/config_overrides_request", "ConfigOverrides"},\
	{"/fmu/in/mode_completed"},\
	{"/fmu/in/register_ext_component_request"},\
	{"/fmu/in/trajectory_setpoint"},\
	{"/fmu/in/unregister_ext_component"},\
	{"/fmu/in/vehicle_command_mode_executor", "VehicleCommand"},\
	{"/fmu/out/arming_check_request"},\
	{"/fmu/out/mode_completed"},\
	{"/fmu/out/register_ext_component_reply"},\
	{"/fmu/out/vehicle_command_ack"},\
	{"/fmu/out/vehicle_status"}


namespace px4_sdk
{

struct MessageCompatibilityTopic {
	std::string topic_name; ///< e.g. "/fmu/out/vehicle_status"
	std::string topic_type{""}; ///< e.g. VehicleStatus. If empty, it's inferred from the topic_name
};

/**
 * Check for a set of messages that the definition matches with the one that PX4 is using.
 * @return true on success
 */
bool messageCompatibilityCheck(rclcpp::Node &node, const std::vector<MessageCompatibilityTopic> &messages_to_check,
							   const std::string &topic_namespace_prefix = "");

} /* namespace px4_sdk */
