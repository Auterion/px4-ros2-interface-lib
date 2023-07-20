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

#include "px4_sdk/components/message_compatibility_check.h"
#include <px4_msgs/msg/message_format_request.hpp>
#include <px4_msgs/msg/message_format_response.hpp>

#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <regex>

namespace px4_sdk {

static std::string messageFieldsStrForMessageHash(rclcpp::Node &node, const std::string& topic_type, const std::string& msgs_dir)
{
	const std::string filename = msgs_dir + "/msg/" + topic_type + ".msg";
	std::ifstream file(filename);
	if (!file.good()) {
		RCLCPP_ERROR(node.get_logger(), "Failed to open %s", filename.c_str());
		return "";
	}
	const std::string text{std::istreambuf_iterator(file), {}};
	std::string fields_str;

	// Match field types from .msg definitions ("foo_msgs/Bar[] bar")
	// See https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html
	static const std::regex kMsgFieldTypeRegex{
			R"((?:^|\n)\s*([a-zA-Z0-9_/]+)(\[[^\]]*\])?\s+(\w+)[ \t]*(=)?)"};

	static const std::set<std::string> kBasicTypes{
			{"bool"}, {"byte"}, {"char"}, {"float32"}, {"float64"}, {"int8"},
			{"uint8"}, {"int16"}, {"uint16"}, {"int32"}, {"uint32"}, {"int64"},
			{"uint64"}, {"string"}, {"wstring"}};

	for (std::sregex_iterator iter(text.begin(), text.end(), kMsgFieldTypeRegex);
		 iter != std::sregex_iterator(); ++iter) {
		const std::string type = (*iter)[1];
		std::string array = (*iter)[2];
		const std::string field_name = (*iter)[3];
		const std::string constant = (*iter)[4];

		if (constant == "=") {
			continue;
		}

		fields_str += type + array + ' ' + field_name + '\n';

		const auto type_iter = kBasicTypes.find(type);
		if (type_iter == kBasicTypes.end()) { // Nested type
			if (type.find('/') == std::string::npos) {
				fields_str += messageFieldsStrForMessageHash(node, type, msgs_dir);
			} else {
				RCLCPP_ERROR(node.get_logger(), "Field %s contains namespace %s", filename.c_str(), type.c_str());
			}
		}
	}

	return fields_str;
}

// source: https://gist.github.com/ruby0x1/81308642d0325fd386237cfa3b44785c
constexpr uint32_t val_32_const = 0x811c9dc5;
constexpr uint32_t prime_32_const = 0x1000193;
inline constexpr uint32_t hash_32_fnv1a_const(const char *const str, const uint32_t value = val_32_const) noexcept
{
	return (str[0] == '\0') ? value : hash_32_fnv1a_const(&str[1], (value ^ uint32_t(str[0])) * prime_32_const);
}

static uint32_t messageHash(rclcpp::Node &node, const std::string& topic_type, const std::string& msgs_dir)
{
	const std::string message_fields_str = messageFieldsStrForMessageHash(node, topic_type, msgs_dir);
	return hash_32_fnv1a_const(message_fields_str.c_str());
}

static std::string snakeToCamelCase(std::string s) noexcept {
	bool tail = false;
	std::size_t n = 0;
	for (unsigned char c : s) {
		if (c == '-' || c == '_') {
			tail = false;
		} else if (tail) {
			s[n++] = c;
		} else {
			tail = true;
			s[n++] = std::toupper(c);
		}
	}
	s.resize(n);
	return s;
}

bool requestMessageFormat(rclcpp::Node &node, const px4_msgs::msg::MessageFormatRequest& request,
						  const rclcpp::Subscription<px4_msgs::msg::MessageFormatResponse>::SharedPtr& message_format_response_sub,
						  const rclcpp::Publisher<px4_msgs::msg::MessageFormatRequest>::SharedPtr& message_format_request_pub,
						  px4_msgs::msg::MessageFormatResponse& response, bool verbose)
{
	rclcpp::WaitSet wait_set;
	wait_set.add_subscription(message_format_response_sub);

	// wait for subscription, it might take a while initially...
	for (int i = 0; i < 100; ++i) {
		if (message_format_request_pub->get_subscription_count() > 0) {
			if (verbose) {
				RCLCPP_DEBUG(node.get_logger(), "Subscriber found, continuing");
			}
			break;
		}

		usleep(100000);
	}

	bool got_reply = false;
	bool had_error = false;
	for (int retries = 0; retries < 5 && !got_reply && !had_error; ++retries) {
		message_format_request_pub->publish(request);

		// wait for publisher, it might take a while initially...
		for (int i = 0; i < 100 && retries == 0; ++i) {
			if (message_format_response_sub->get_publisher_count() > 0) {
				if (verbose) {
					RCLCPP_DEBUG(node.get_logger(), "Publisher found, continuing");
				}
				break;
			}

			usleep(100000);
		}

		auto start_time = std::chrono::steady_clock::now();
		auto timeout = 300ms;

		while (!got_reply && !had_error) {
			auto now = std::chrono::steady_clock::now();

			if (now >= start_time + timeout) {
				break;
			}

			auto wait_ret = wait_set.wait(timeout - (now - start_time));

			if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
				rclcpp::MessageInfo info;

				if (message_format_response_sub->take(response, info)) {

					if (response.protocol_version == px4_msgs::msg::MessageFormatRequest::LATEST_PROTOCOL_VERSION) {
						response.topic_name.back() = 0;
						if (strcmp((const char *) response.topic_name.data(), (const char *)request.topic_name.data()) == 0) {
							got_reply = true;
						} // Else: response to a different topic, try again
					} else {
						RCLCPP_ERROR(node.get_logger(), "Protocol version mismatch: got %i, expected %i", response.protocol_version, px4_msgs::msg::MessageFormatRequest::LATEST_PROTOCOL_VERSION);
						had_error = true;
					}

				} else {
					RCLCPP_INFO(node.get_logger(), "no message received");
				}

			} else {
				RCLCPP_INFO(node.get_logger(), "timeout");
			}
		}
	}
	wait_set.remove_subscription(message_format_response_sub);
	return got_reply;
}

bool messageCompatibilityCheck(rclcpp::Node &node, const std::vector<MessageCompatibilityTopic> &messages_to_check,
							   const std::string &topic_namespace_prefix)
{
	RCLCPP_DEBUG(node.get_logger(), "Checking message compatibility...");
	rclcpp::Subscription<px4_msgs::msg::MessageFormatResponse>::SharedPtr message_format_response_sub =
			node.create_subscription<px4_msgs::msg::MessageFormatResponse>(
					topic_namespace_prefix + "/fmu/out/message_format_response", rclcpp::QoS(1).best_effort(),
					[](px4_msgs::msg::MessageFormatResponse::UniquePtr msg) {});

	rclcpp::Publisher<px4_msgs::msg::MessageFormatRequest>::SharedPtr message_format_request_pub =
			node.create_publisher<px4_msgs::msg::MessageFormatRequest>(
			topic_namespace_prefix + "/fmu/in/message_format_request", 1);

	const std::string msgs_dir = ament_index_cpp::get_package_share_directory("px4_msgs");
	if (msgs_dir.empty()) {
		RCLCPP_FATAL(node.get_logger(), "Failed to get installation directory for 'px4_msgs' package");
		return false;
	}

	bool ret = true;
	std::string mismatched_topics;
	bool first_message = true;

	for (const auto& message_to_check : messages_to_check) {
		std::string topic_type = message_to_check.topic_type;
		if (topic_type.empty()) {
			// Infer topic type from topic_name
			auto last_slash = message_to_check.topic_name.find_last_of('/');
			if (last_slash == std::string::npos) {
				topic_type = message_to_check.topic_name;
			} else {
				topic_type = message_to_check.topic_name.substr(last_slash + 1);
			}
			topic_type = snakeToCamelCase(topic_type);
		}
		// Read the local message definition and get the hash
		const uint32_t expected_message_hash = messageHash(node, topic_type, msgs_dir);

		// Ask for the message hash from PX4
		px4_msgs::msg::MessageFormatRequest request;
		request.protocol_version = px4_msgs::msg::MessageFormatRequest::LATEST_PROTOCOL_VERSION;
		strncpy((char*)request.topic_name.data(), message_to_check.topic_name.c_str(), request.topic_name.size());
		request.timestamp = node.get_clock()->now().nanoseconds() / 1000;
		px4_msgs::msg::MessageFormatResponse response;
		if (requestMessageFormat(node, request, message_format_response_sub, message_format_request_pub, response, first_message)) {
			if (response.success) {
				if (response.message_hash != expected_message_hash) {
					mismatched_topics += "\n  - " + message_to_check.topic_name;
					ret = false;
				}
			} else {
				RCLCPP_FATAL(node.get_logger(), "MessageFormatResponse::success == false");
				ret = false;
			}
		} else {
			RCLCPP_FATAL(node.get_logger(), "No response to MessageFormatResponse");
			ret = false;
		}
		first_message = false;
	}

	if (!mismatched_topics.empty()) {
		RCLCPP_ERROR(node.get_logger(), "Mismatch for the following topics, update PX4 or the SDK and px4_msgs:%s", mismatched_topics.c_str());
	}

	return ret;
}

} // namespace px4_sdk