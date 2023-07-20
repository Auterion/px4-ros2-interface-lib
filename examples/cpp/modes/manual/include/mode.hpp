/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_sdk/components/mode.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals; // NOLINT

static const char * name = "My Manual Mode";
static const char * node_name = "example_mode_manual";

static inline Eigen::Quaterniond quaternionFromEuler(const Eigen::Vector3d & euler)
{
  // YPR is ZYX axes
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()));
}

static inline Eigen::Quaterniond quaternionFromEuler(
  const double roll, const double pitch,
  const double yaw)
{
  return quaternionFromEuler(Eigen::Vector3d(roll, pitch, yaw));
}

class FlightModeTest : public px4_sdk::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, Settings{name}, px4_sdk::ModeRequirements::manualControlledPosition())
  {
    _manual_control_setpoint_sub = node.create_subscription<px4_msgs::msg::ManualControlSetpoint>(
      "/fmu/out/manual_control_setpoint", rclcpp::QoS(1).best_effort(),
      [this, &node](px4_msgs::msg::ManualControlSetpoint::UniquePtr msg) {
        _manual_control_setpoint = *msg;
        _last_manual_control_setpoint = node.get_clock()->now();
      });
    _vehicle_rates_setpoint_pub = node.create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
      "/fmu/in/vehicle_rates_setpoint", 1);
    _vehicle_attitude_setpoint_pub = node.create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
      "/fmu/in/vehicle_attitude_setpoint", 1);

    setSetpointUpdateRate(60.F);
    _last_manual_control_setpoint = node.get_clock()->now();
  }

  void onActivate() override
  {
    _activation_time = node().get_clock()->now();
    _last_update = _activation_time;

    _config.manual_enabled = false;
    _config.auto_enabled = false;
    _config.rates_enabled = true;
    _config.attitude_enabled = true;
    _config.acceleration_enabled = false;
    _config.velocity_enabled = false;
    _config.position_enabled = false;
    _config.altitude_enabled = false;
    setpoints().configureSetpointsSync(_config);
  }

  void onDeactivate() override {}

  void updateSetpoint() override
  {
    const rclcpp::Time now = node().get_clock()->now();

    const float threshold = 0.9F;
    const bool want_rates = fabs(_manual_control_setpoint.roll) > threshold || fabsf(
      _manual_control_setpoint.pitch) > threshold;

    if (_config.attitude_enabled == want_rates) {
      _config.attitude_enabled = !want_rates;
      setpoints().configureSetpointsSync(_config);
    }

    const float dt = (now - _last_update).seconds();

    const float yaw_rate = _manual_control_setpoint.yaw * 120.F * M_PI / 180.F;

    if (want_rates) {
      px4_msgs::msg::VehicleRatesSetpoint sp{};
      sp.thrust_body[2] = -_manual_control_setpoint.throttle;
      sp.yaw = yaw_rate;
      sp.roll = _manual_control_setpoint.roll * 500.F * M_PI / 180.F;
      sp.pitch = -_manual_control_setpoint.pitch * 500.F * M_PI / 180.F;
      sp.timestamp = node().get_clock()->now().nanoseconds() / 1000;
      _vehicle_rates_setpoint_pub->publish(sp);

    } else {
      _yaw += yaw_rate * dt;
      px4_msgs::msg::VehicleAttitudeSetpoint sp{};
      sp.thrust_body[2] = -_manual_control_setpoint.throttle;
      sp.yaw_sp_move_rate = yaw_rate;
      Eigen::Quaterniond qd = quaternionFromEuler(
        _manual_control_setpoint.roll * 55.F * M_PI / 180.F,
        -_manual_control_setpoint.pitch * 55.F * M_PI / 180.F,
        _yaw
      );
      sp.q_d[0] = qd.w();
      sp.q_d[1] = qd.x();
      sp.q_d[2] = qd.y();
      sp.q_d[3] = qd.z();
      sp.timestamp = node().get_clock()->now().nanoseconds() / 1000;
      _vehicle_attitude_setpoint_pub->publish(sp);
    }

    _last_update = now;
  }

private:
  rclcpp::Time _activation_time{};
  rclcpp::Time _last_update{};
  rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr _manual_control_setpoint_sub;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr
    _vehicle_attitude_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr _vehicle_rates_setpoint_pub;
  px4_msgs::msg::ManualControlSetpoint _manual_control_setpoint{};
  rclcpp::Time _last_manual_control_setpoint{};
  float _yaw{0.F};
  px4_sdk::SetpointSender::SetpointConfiguration _config{};
};

class TestNode : public rclcpp::Node
{
public:
  TestNode()
  : Node(node_name)
  {
    // Enable debug output
    auto ret =
      rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
    }

    _mode = std::make_unique<FlightModeTest>(*this);

    if (!_mode->doRegister()) {
      throw std::runtime_error("Registration failed");
    }
  }

private:
  std::unique_ptr<FlightModeTest> _mode;
};
