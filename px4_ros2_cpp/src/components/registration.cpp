/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "registration.hpp"

#include <cassert>
#include <random>
#include <unistd.h>
#include <px4_ros2/utils/message_version.hpp>

static constexpr uint16_t kLatestPX4ROS2ApiVersion = 1;

using namespace std::chrono_literals;

Registration::Registration(rclcpp::Node & node, const std::string & topic_namespace_prefix)
: _node(node)
{
  _register_ext_component_reply_sub =
    node.create_subscription<px4_msgs::msg::RegisterExtComponentReply>(
    topic_namespace_prefix + "fmu/out/register_ext_component_reply" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::RegisterExtComponentReply>(),
    rclcpp::QoS(1).best_effort(),
    [](px4_msgs::msg::RegisterExtComponentReply::UniquePtr msg) {
    });

  _register_ext_component_request_pub =
    node.create_publisher<px4_msgs::msg::RegisterExtComponentRequest>(
    topic_namespace_prefix + "fmu/in/register_ext_component_request" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::RegisterExtComponentRequest>(),
    1);

  _unregister_ext_component_pub = node.create_publisher<px4_msgs::msg::UnregisterExtComponent>(
    topic_namespace_prefix + "fmu/in/unregister_ext_component" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::UnregisterExtComponent>(),
    1);

  _unregister_ext_component.mode_id = px4_ros2::ModeBase::kModeIDInvalid;
}

bool Registration::doRegister(const RegistrationSettings & settings)
{
  assert(!_registered);
  px4_msgs::msg::RegisterExtComponentRequest request{};

  if (settings.name.length() >= request.name.size() ||
    settings.name.length() >= _unregister_ext_component.name.size())
  {
    RCLCPP_ERROR(
      _node.get_logger(), "Name too long (%i >= %i)",
      (int)settings.name.length(), (int)request.name.size());
    return false;
  }

  RCLCPP_DEBUG(
    _node.get_logger(), "Registering '%s' (arming check: %i, mode: %i, mode executor: %i)",
    settings.name.c_str(),
    settings.register_arming_check, settings.register_mode, settings.register_mode_executor);

  strcpy(reinterpret_cast<char *>(request.name.data()), settings.name.c_str());
  request.register_arming_check = settings.register_arming_check;
  request.register_mode = settings.register_mode;
  request.register_mode_executor = settings.register_mode_executor;
  request.enable_replace_internal_mode = settings.enable_replace_internal_mode;
  request.replace_internal_mode = settings.replace_internal_mode;
  request.activate_mode_immediately = settings.activate_mode_immediately;
  request.px4_ros2_api_version = kLatestPX4ROS2ApiVersion;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint64_t> distrib{};
  request.request_id = distrib(gen);

  // wait for subscription, it might take a while initially...
  for (int i = 0; i < 100; ++i) {
    if (_register_ext_component_request_pub->get_subscription_count() > 0) {
      RCLCPP_DEBUG(_node.get_logger(), "Subscriber found, continuing");
      break;
    }

    usleep(100000);
  }

  // send request and wait for response
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(_register_ext_component_reply_sub);

  bool got_reply = false;

  for (int retries = 0; retries < 5 && !got_reply; ++retries) {
    request.timestamp = 0; // Let PX4 set the timestamp
    _register_ext_component_request_pub->publish(request);

    // wait for publisher, it might take a while initially...
    for (int i = 0; i < 100 && retries == 0; ++i) {
      if (_register_ext_component_reply_sub->get_publisher_count() > 0) {
        RCLCPP_DEBUG(_node.get_logger(), "Publisher found, continuing");
        break;
      }

      usleep(100000);
    }

    const auto start_time = std::chrono::steady_clock::now();
    const auto timeout = 1000ms; // CI simulation tests require this to be quite high

    while (!got_reply) {
      auto now = std::chrono::steady_clock::now();

      if (now >= start_time + timeout) {
        break;
      }

      auto wait_ret = wait_set.wait(timeout - (now - start_time));

      if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
        px4_msgs::msg::RegisterExtComponentReply reply;
        rclcpp::MessageInfo info;

        if (_register_ext_component_reply_sub->take(reply, info)) {
          reply.name.back() = '\0';

          if (strcmp(
              reinterpret_cast<const char *>(reply.name.data()),
              settings.name.c_str()) == 0 &&
            request.request_id == reply.request_id)
          {
            RCLCPP_DEBUG(_node.get_logger(), "Got RegisterExtComponentReply");

            if (reply.success) {
              if (reply.px4_ros2_api_version == kLatestPX4ROS2ApiVersion) {
                _unregister_ext_component.arming_check_id = reply.arming_check_id;
                _unregister_ext_component.mode_id = reply.mode_id;
                _unregister_ext_component.mode_executor_id = reply.mode_executor_id;
                strcpy(
                  reinterpret_cast<char *>(_unregister_ext_component.name.data()),
                  settings.name.c_str());
                _registered = true;
              } else {
                RCLCPP_FATAL(
                  _node.get_logger(), "Incompatible ROS2 library API version: got %i, expected %i",
                  reply.px4_ros2_api_version, kLatestPX4ROS2ApiVersion);
              }

            } else {
              RCLCPP_ERROR(_node.get_logger(), "Registration failed");
            }

            got_reply = true;
          }

        } else {
          RCLCPP_INFO(_node.get_logger(), "no RegisterExtComponentReply message received");
        }

      } else {
        RCLCPP_INFO(_node.get_logger(), "timeout while registering external component");
      }
    }
  }

  wait_set.remove_subscription(_register_ext_component_reply_sub);

  return _registered;
}

void Registration::doUnregister()
{
  if (_registered) {
    RCLCPP_DEBUG(_node.get_logger(), "Unregistering");
    _unregister_ext_component.timestamp = 0; // Let PX4 set the timestamp
    _unregister_ext_component_pub->publish(_unregister_ext_component);
    _registered = false;
  }
}

void Registration::setRegistrationDetails(
  int arming_check_id, px4_ros2::ModeBase::ModeID mode_id,
  int mode_executor_id)
{
  _unregister_ext_component.arming_check_id = arming_check_id;
  _unregister_ext_component.mode_id = mode_id;
  _unregister_ext_component.mode_executor_id = mode_executor_id;
  _registered = true;
}
