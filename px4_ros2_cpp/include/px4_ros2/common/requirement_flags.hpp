/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/arming_check_reply.hpp>

namespace px4_ros2
{

/**
 * @brief Requirement flags used by modes
 */
struct RequirementFlags
{
  void fillArmingCheckReply(px4_msgs::msg::ArmingCheckReply & arming_check_reply)
  {
    arming_check_reply.mode_req_angular_velocity = angular_velocity;
    arming_check_reply.mode_req_attitude = attitude;
    arming_check_reply.mode_req_local_alt = local_alt;
    arming_check_reply.mode_req_local_position = local_position;
    arming_check_reply.mode_req_local_position_relaxed = local_position_relaxed;
    arming_check_reply.mode_req_global_position = global_position;
    arming_check_reply.mode_req_mission = mission;
    arming_check_reply.mode_req_home_position = home_position;
    arming_check_reply.mode_req_prevent_arming = prevent_arming;
    arming_check_reply.mode_req_manual_control = manual_control;
  }

  void clearAll()
  {
    *this = RequirementFlags{};
  }

  RequirementFlags & operator|=(const RequirementFlags & other)
  {
    angular_velocity |= other.angular_velocity;
    attitude |= other.attitude;
    local_alt |= other.local_alt;
    local_position |= other.local_position;
    local_position_relaxed |= other.local_position_relaxed;
    global_position |= other.global_position;
    mission |= other.mission;
    home_position |= other.home_position;
    prevent_arming |= other.prevent_arming;
    manual_control |= other.manual_control;
    return *this;
  }

  bool angular_velocity{false};
  bool attitude{false};
  bool local_alt{false};
  bool local_position{false};
  bool local_position_relaxed{false};
  bool global_position{false};
  bool mission{false};
  bool home_position{false};
  bool prevent_arming{false}; ///< If set, arming is prevented when the mode is selected
  bool manual_control{false};
};

} // namespace px4_ros2
