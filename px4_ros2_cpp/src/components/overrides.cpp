/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "px4_ros2/components/overrides.hpp"
#include "px4_ros2/utils/message_version.hpp"

#include <cassert>

namespace px4_ros2
{

ConfigOverrides::ConfigOverrides(rclcpp::Node & node, const std::string & topic_namespace_prefix)
: _node(node)
{
  _config_overrides_pub = _node.create_publisher<px4_msgs::msg::ConfigOverrides>(
    topic_namespace_prefix + "fmu/in/config_overrides_request" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::ConfigOverrides>(),
    1);
}

void ConfigOverrides::controlAutoDisarm(bool enabled)
{
  _current_overrides.disable_auto_disarm = !enabled;
  update();
}

void ConfigOverrides::deferFailsafes(bool enabled, int timeout_s)
{
  _current_overrides.defer_failsafes = enabled;
  _current_overrides.defer_failsafes_timeout_s = timeout_s;
  update();
}

void ConfigOverrides::update()
{
  if (_is_setup) {
    _current_overrides.timestamp = 0; // Let PX4 set the timestamp
    _config_overrides_pub->publish(_current_overrides);

  } else {
    _require_update_after_setup = true;
  }
}

void ConfigOverrides::setup(uint8_t type, uint8_t id)
{
  assert(!_is_setup);
  _current_overrides.source_type = type;
  _current_overrides.source_id = id;
  _is_setup = true;

  if (_require_update_after_setup) {
    update();
    _require_update_after_setup = false;
  }
}
} // namespace px4_ros2
