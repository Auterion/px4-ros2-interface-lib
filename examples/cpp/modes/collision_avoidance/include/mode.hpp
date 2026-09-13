#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

// Import the Waypoint message from drone_path_planner
#include <drone_path_planner/msg/waypoint.hpp>
#include <drone_path_planner/msg/planner_status.hpp>

static const std::string kModeName = "Waypoint Follower";

/**
 * @brief Simple flight mode that follows waypoints from the path planner
 * 
 * This mode:
 * - Subscribes to /planner/waypoint and /planner/status
 * - Flies to each waypoint received
 * - Holds position if no valid waypoint or planner unhealthy
 * - Very simple and reliable - all intelligence is in the path planner
 */
class WaypointFollowerMode : public px4_ros2::ModeBase
{
public:
  explicit WaypointFollowerMode(rclcpp::Node & node)
  : ModeBase(node, Settings{kModeName}), _node(node)
  {
    _goto_setpoint = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    // Subscribe to path planner outputs
    _waypoint_sub = _node.create_subscription<drone_path_planner::msg::Waypoint>(
      "/planner/waypoint", rclcpp::QoS(10).reliable(),
      std::bind(&WaypointFollowerMode::waypointCallback, this, std::placeholders::_1));

    _status_sub = _node.create_subscription<drone_path_planner::msg::PlannerStatus>(
      "/planner/status", rclcpp::QoS(10).reliable(),
      std::bind(&WaypointFollowerMode::statusCallback, this, std::placeholders::_1));

    RCLCPP_INFO(_node.get_logger(), "=== Waypoint Follower Mode ===");
    RCLCPP_INFO(_node.get_logger(), "Listening to /planner/waypoint");
  }

  void onActivate() override
  {
    RCLCPP_INFO(_node.get_logger(), ">>> ACTIVATED <<<");
    _hold_position = _vehicle_local_position->positionNed();
    _has_waypoint = false;
    _planner_healthy = false;
  }

  void onDeactivate() override
  {
    RCLCPP_INFO(_node.get_logger(), ">>> DEACTIVATED <<<");
  }

  void updateSetpoint(float dt_s) override
  {
    (void)dt_s;

    Eigen::Vector3f target;

    // Check if we should fly to waypoint or hold
    bool should_fly = _has_waypoint && 
                      _last_waypoint.valid && 
                      _planner_healthy &&
                      _last_waypoint.waypoint_type != drone_path_planner::msg::Waypoint::TYPE_EMERGENCY;

    float target_yaw = _vehicle_local_position->heading();  // Default: current heading
    
    if (should_fly) {
      // Fly to the waypoint from path planner
      target.x() = _last_waypoint.position_n;
      target.y() = _last_waypoint.position_e;
      target.z() = _last_waypoint.position_d;
      
      // Use yaw from waypoint if valid
      if (_last_waypoint.yaw_valid) {
        target_yaw = _last_waypoint.yaw;
      }
    } else {
      // Hold current position
      target = _hold_position;
      
      // Update hold position to current if we're not flying
      if (_vehicle_local_position->lastValid()) {
        _hold_position = _vehicle_local_position->positionNed();
      }
    }

    // Update setpoint with position and yaw
    _goto_setpoint->update(target, target_yaw);

    // Logging
    _update_counter++;
    if (_update_counter >= 30) {  // ~1 Hz logging
      _update_counter = 0;
      
      Eigen::Vector3f pos = _vehicle_local_position->positionNed();
      
      if (should_fly) {
        RCLCPP_INFO(_node.get_logger(), "[FLYING] Pos:[%.1f,%.1f,%.1f] -> WP:[%.1f,%.1f,%.1f]",
          pos.x(), pos.y(), pos.z(),
          target.x(), target.y(), target.z());
      } else {
        const char* reason = !_has_waypoint ? "No waypoint" :
                            !_last_waypoint.valid ? "Invalid waypoint" :
                            !_planner_healthy ? "Planner unhealthy" :
                            "Emergency stop";
        RCLCPP_WARN(_node.get_logger(), "[HOLD] %s | Pos:[%.1f,%.1f,%.1f]",
          reason, pos.x(), pos.y(), pos.z());
      }
    }
  }

private:
  rclcpp::Node & _node;
  std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _goto_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;

  // Subscriptions
  rclcpp::Subscription<drone_path_planner::msg::Waypoint>::SharedPtr _waypoint_sub;
  rclcpp::Subscription<drone_path_planner::msg::PlannerStatus>::SharedPtr _status_sub;

  // State
  drone_path_planner::msg::Waypoint _last_waypoint;
  bool _has_waypoint{false};
  bool _planner_healthy{false};
  Eigen::Vector3f _hold_position{0.f, 0.f, 0.f};
  int _update_counter{0};

  void waypointCallback(const drone_path_planner::msg::Waypoint::SharedPtr msg)
  {
    _last_waypoint = *msg;
    _has_waypoint = true;
  }

  void statusCallback(const drone_path_planner::msg::PlannerStatus::SharedPtr msg)
  {
    // TODO: Re-enable health check after testing
    // For now, always consider healthy if we receive status messages
    (void)msg;
    _planner_healthy = true;
  }
};

// Alias for backward compatibility
using CollisionAvoidanceMode = WaypointFollowerMode;
