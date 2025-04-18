/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "px4_ros2/components/message_compatibility_check.hpp"
#include <px4_msgs/msg/message_format_request.hpp>
#include <px4_msgs/msg/message_format_response.hpp>
#include <px4_ros2/utils/message_version.hpp>

#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <regex>
#include <unistd.h>

namespace
{
std::string messageFieldsStrForMessageHash(
  rclcpp::Node & node,
  const std::string & topic_type,
  const std::string & msgs_dir)
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
    {"bool"},
    {"byte"},
    {"char"},
    {"float32"},
    {"float64"},
    {"int8"},
    {"uint8"},
    {"int16"},
    {"uint16"},
    {"int32"},
    {"uint32"},
    {"int64"},
    {"uint64"},
    {"string"},
    {"wstring"}};

  for (std::sregex_iterator iter(text.begin(), text.end(), kMsgFieldTypeRegex);
    iter != std::sregex_iterator(); ++iter)
  {
    const std::string type = (*iter)[1];
    const std::string array = (*iter)[2];
    const std::string field_name = (*iter)[3];
    const std::string constant = (*iter)[4];

    if (constant == "=") {
      continue;
    }

    fields_str += type + array + ' ' + field_name + '\n'; // NOLINT

    const auto type_iter = kBasicTypes.find(type);
    if (type_iter == kBasicTypes.end()) {             // Nested type
      if (type.find('/') == std::string::npos) {
        fields_str += messageFieldsStrForMessageHash(node, type, msgs_dir);
      } else {
        RCLCPP_ERROR(
          node.get_logger(), "Field %s contains namespace %s",
          filename.c_str(), type.c_str());
      }
    }
  }

  return fields_str;
}

// source: https://gist.github.com/ruby0x1/81308642d0325fd386237cfa3b44785c
constexpr uint32_t kVal32Const = 0x811c9dc5;
constexpr uint32_t kPrime32Const = 0x1000193;

inline constexpr uint32_t hash32Fnv1aConst(
  const char * const str,
  const uint32_t value = kVal32Const) noexcept
{
  return (str[0] == '\0') ? value : hash32Fnv1aConst(
    &str[1], (value ^ static_cast<uint32_t>(
      str[0])) * kPrime32Const);
}

uint32_t messageHash(
  rclcpp::Node & node, const std::string & topic_type,
  const std::string & msgs_dir)
{
  const std::string message_fields_str = messageFieldsStrForMessageHash(node, topic_type, msgs_dir);
  return hash32Fnv1aConst(message_fields_str.c_str());
}

std::string snakeToCamelCase(std::string s) noexcept
{
  bool tail = false;
  std::size_t n = 0;
  for (const unsigned char c: s) {
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

enum class RequestMessageFormatReturn
{
  GotReply,
  Timeout,
  ProtocolVersionMismatch,
};

RequestMessageFormatReturn requestMessageFormat(
  rclcpp::Node & node, const px4_msgs::msg::MessageFormatRequest & request,
  const rclcpp::Subscription<px4_msgs::msg::MessageFormatResponse>::SharedPtr &
  message_format_response_sub,
  const rclcpp::Publisher<px4_msgs::msg::MessageFormatRequest>::SharedPtr &
  message_format_request_pub,
  px4_msgs::msg::MessageFormatResponse & response, bool verbose)
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

  RequestMessageFormatReturn request_message_format_return{RequestMessageFormatReturn::Timeout};
  for (int retries = 0;
    retries < 5 && request_message_format_return == RequestMessageFormatReturn::Timeout;
    ++retries)
  {
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

    while (request_message_format_return == RequestMessageFormatReturn::Timeout) {
      auto now = std::chrono::steady_clock::now();

      if (now >= start_time + timeout) {
        break;
      }

      auto wait_ret = wait_set.wait(timeout - (now - start_time));

      if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
        rclcpp::MessageInfo info;

        if (message_format_response_sub->take(response, info)) {

          if (response.protocol_version ==
            px4_msgs::msg::MessageFormatRequest::LATEST_PROTOCOL_VERSION)
          {
            response.topic_name.back() = 0;
            if (strcmp(
                reinterpret_cast<const char *>(response.topic_name.data()),
                reinterpret_cast<const char *>(request.topic_name.data())) == 0)
            {
              request_message_format_return = RequestMessageFormatReturn::GotReply;
            }                                     // Else: response to a different topic, try again
          } else {
            RCLCPP_ERROR(
              node.get_logger(), "Protocol version mismatch: got %i, expected %i",
              response.protocol_version,
              px4_msgs::msg::MessageFormatRequest::LATEST_PROTOCOL_VERSION);
            request_message_format_return = RequestMessageFormatReturn::ProtocolVersionMismatch;
          }

        } else {
          RCLCPP_INFO(node.get_logger(), "no MessageFormatResponse message received");
        }

      } else {
        RCLCPP_INFO(node.get_logger(), "timeout while checking message compatibility");
      }
    }
  }
  wait_set.remove_subscription(message_format_response_sub);
  return request_message_format_return;
}

} // namespace

namespace px4_ros2
{

bool messageCompatibilityCheck(
  rclcpp::Node & node, const std::vector<MessageCompatibilityTopic> & messages_to_check,
  const std::string & topic_namespace_prefix)
{
  RCLCPP_DEBUG(node.get_logger(), "Checking message compatibility...");
  const rclcpp::Subscription<px4_msgs::msg::MessageFormatResponse>::SharedPtr
    message_format_response_sub
    =
    node.create_subscription<px4_msgs::msg::MessageFormatResponse>(
      topic_namespace_prefix + "fmu/out/message_format_response" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::MessageFormatResponse>(), rclcpp::QoS(
        1).best_effort(),
      [](px4_msgs::msg::MessageFormatResponse::UniquePtr msg) {});

  const rclcpp::Publisher<px4_msgs::msg::MessageFormatRequest>::SharedPtr message_format_request_pub
    =
    node.create_publisher<px4_msgs::msg::MessageFormatRequest>(
      topic_namespace_prefix + "fmu/in/message_format_request" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::MessageFormatRequest>(),
      1);

  const std::string msgs_dir = ament_index_cpp::get_package_share_directory("px4_msgs");
  if (msgs_dir.empty()) {
    RCLCPP_FATAL(node.get_logger(), "Failed to get installation directory for 'px4_msgs' package");
    return false;
  }

  bool ret = true;
  std::string mismatched_topics;
  bool first_message = true;

  for (const auto & message_to_check : messages_to_check) {
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
    strncpy(
      reinterpret_cast<char *>(request.topic_name.data()),
      message_to_check.topic_name.c_str(), request.topic_name.size() - 1);
    request.topic_name.back() = '\0';
    request.timestamp = 0; // Let PX4 set the timestamp
    px4_msgs::msg::MessageFormatResponse response;
    switch (requestMessageFormat(
        node, request, message_format_response_sub, message_format_request_pub,
        response, first_message))
    {
      case RequestMessageFormatReturn::Timeout:
        RCLCPP_FATAL(
          node.get_logger(),
          "Timed out waiting for message format. Is the FMU running?");
        // Do not try to check the other formats
        return false;
      case RequestMessageFormatReturn::ProtocolVersionMismatch:
        // Error already reported
        return false;
      case RequestMessageFormatReturn::GotReply:
        if (response.success) {
          if (response.message_hash != expected_message_hash) {
            mismatched_topics += "\n  - " + message_to_check.topic_name;
            ret = false;
          }
        } else {
          RCLCPP_FATAL(node.get_logger(), "MessageFormatResponse::success == false");
          ret = false;
        }
        break;
    }
    first_message = false;
  }

  if (!mismatched_topics.empty()) {
    RCLCPP_ERROR(
      node.get_logger(),
      "Mismatch for the following topics, update PX4 or the px4_ros2 library and px4_msgs:%s",
      mismatched_topics.c_str());
  }

  return ret;
}

} // namespace px4_ros2
