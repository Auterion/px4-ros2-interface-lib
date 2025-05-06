/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <algorithm>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>


using namespace std::chrono_literals; // NOLINT

static const std::string kName = "Autonomous Acro";

static constexpr float kOneG = 9.80665f;

class ThrustController
{
public:
  ThrustController(
    rclcpp::Node & node, float & p_gain, float & d_gain, float & radius,
    const std::shared_ptr<px4_ros2::OdometryLocalPosition> & local_position)
  : _node(node), _p_gain(p_gain), _d_gain(d_gain), _radius(radius), _local_position(local_position)
  {
    // Debug topics
    _center_pub = node.create_publisher<geometry_msgs::msg::Vector3>("~/center", 1);
    _point_on_loop_pub =
      node.create_publisher<geometry_msgs::msg::Vector3>("~/point_on_loop", 1);
    _pid_pub = node.create_publisher<geometry_msgs::msg::Vector3>("~/pid", 1);

    _last_update = _node.get_clock()->now();
    // Run the controller whenever we get a position update
    local_position->onUpdate(
      [this](const px4_msgs::msg::VehicleLocalPosition & local_position) {
        update();
      });
  }

  void reset()
  {
    // We set the rotation center above the current position. This assumes the vehicle is currently level,
    // which is not quite true as it has some forward velocity.
    _center = _local_position->positionNed();
    _center(2) -= _radius;

    // This is the plan normal in which the vehicle rotates
    _rotation_plane_normal =
      Eigen::AngleAxisf(
      _local_position->heading(),
      Eigen::Vector3f::UnitZ()) * Eigen::Vector3f::UnitY();
    _last_error = 0.0f;
    _last_update = _node.get_clock()->now();
    _controller_output = 0.0f;
  }

  float controllerOutput() const {return _controller_output;}

  template<typename Pub>
  void publishVector(Pub & pub, const Eigen::Vector3f & vector)
  {
    geometry_msgs::msg::Vector3 msg;
    msg.x = vector(0);
    msg.y = vector(1);
    msg.z = vector(2);
    pub->publish(msg);
  }

private:
  void update()
  {
    // Rescale the gains with the radius. The gains need to be higher for smaller radii
    const float p_gain = _p_gain / _radius;
    const float d_gain = _d_gain / _radius;

    // As the vehicle can shift sidewards with respect to the rotation plane, we need to shift the center of the loop
    const Eigen::Vector3f shifted_center = _center - (_center - _local_position->positionNed()).dot(
      _rotation_plane_normal) * _rotation_plane_normal;
    const float error = (_local_position->positionNed() - shifted_center).norm() - _radius;
    const float p_term = p_gain * error;
    const auto now = _node.get_clock()->now();
    const float d_term = d_gain * (error - _last_error) / (now - _last_update).seconds();
    _last_error = error;
    _last_update = now;

    // Debug topics
    publishVector(_center_pub, shifted_center);
    const Eigen::Vector3f point_on_loop = shifted_center +
      (_local_position->positionNed() - shifted_center).normalized() * _radius;
    publishVector(_point_on_loop_pub, point_on_loop);
    publishVector(_pid_pub, Eigen::Vector3f(p_term, 0.0f, d_term));

    _controller_output = p_term + d_term;
  }

  rclcpp::Node & _node;
  const float & _p_gain;
  const float & _d_gain;
  const float & _radius;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_position;
  Eigen::Vector3f _center{0.f, 0.f, 0.f};
  Eigen::Vector3f _rotation_plane_normal{};
  float _last_error{0.0f};
  rclcpp::Time _last_update{};
  float _controller_output{0.0f};

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr _center_pub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr _point_on_loop_pub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr _pid_pub;
};

class FlightModeTest : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, kName)
  {
    _manual_control_input = std::make_shared<px4_ros2::ManualControlInput>(*this, true);
    _rates_setpoint = std::make_shared<px4_ros2::RatesSetpointType>(*this);
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

    // Defaults
    _radius = 5.0f;
    _velocity = optimalVelocityFromRadius(_radius);
    _hover_thrust = 0.27f; // This is model-specific
    _attitude_delay = 0.02f;
    _thrust_p_gain = 0.16f;
    _thrust_d_gain = 0.004f;

    // We increase the start target velocity from what actually would be needed by a certain factor.
    // Because of the gravity term, the velocity on the loop is not constant, but is highest
    // at the lowest point (where we start). Testing with different velocities and radii, the factor
    // was always about the same, so this should work in general.
    _start_velocity_adjustment = 1.6f;

    _thrust_controller = std::make_shared<ThrustController>(
      node, _thrust_p_gain, _thrust_d_gain, _radius, _local_position);

    // Parameters
    _param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(&node);
    declareParameter("hover_thrust", _hover_thrust);
    declareParameter("radius_m", _radius);
    declareParameter("velocity_m_s", _velocity);
    declareParameter("attitude_delay_s", _attitude_delay, true);
    declareParameter("thrust_p_gain", _thrust_p_gain, true);
    declareParameter("thrust_d_gain", _thrust_d_gain, true);
  }

  template<typename T>
  void declareParameter(const std::string & name, T & value, bool allow_update_while_active = false)
  {
    node().declare_parameter(name, value);
    _cb_handles.emplace_back(
      _param_subscriber->add_parameter_callback(
        name, [this, &value, allow_update_while_active](const rclcpp::Parameter & p) {
          if (allow_update_while_active || !isActive()) {
            value = p.get_value<T>();
          }
        }));
  }


  void onActivate() override
  {
    _state = State::BuildupVelocity;
  }

  void onDeactivate() override {}

  static float optimalVelocityFromRadius(float radius)
  {
    // Calculate a velocity that satisfies the thrust feasibility requirements.
    // There's more options with a lower hover thrust, this is just the most versatile.
    return std::sqrt(2.0f * kOneG * radius);
  }

  void updateSetpoint(float dt_s) override
  {
    switch (_state) {
      case State::BuildupVelocity:
        {
          const auto heading = _local_position->heading();
          const auto target_velocity = _start_velocity_adjustment * _velocity;
          Eigen::Vector3f velocity_sp{target_velocity * 1.2f, 0, 0};
          // Rotate velocity into body frame with current heading
          velocity_sp = Eigen::AngleAxisf(heading, Eigen::Vector3f::UnitZ()) * velocity_sp;
          _trajectory_setpoint->update(velocity_sp);

          // Check if we reached the velocity
          if (_local_position->velocityNed().head<2>().norm() > target_velocity) {
            RCLCPP_INFO(node().get_logger(), "Target velocity reached (%.1f m/s)", target_velocity);
            _state = State::RotateToZero;
          }
        }
        break;
      case State::RotateToZero:
      case State::Powerloop:
        {
          // We use a fixed angular rate and adjust the thrust accordingly
          const float angular_rate = _velocity / _radius;

          // Centripetal force
          const float mass = _hover_thrust / kOneG;
          const float acceleration = angular_rate * angular_rate * _radius;
          float centripetal_force = mass * acceleration;

          // Gravity compensation
          const Eigen::Vector3f gravity_vector{0, 0, kOneG};
          // Rotate into body frame using the attitude estimate
          auto attitude_rpy = px4_ros2::quaternionToEulerRpy(_attitude->attitude());
          attitude_rpy(1) += angular_rate * _attitude_delay;
          const Eigen::Vector3f centripetal_acceleration = px4_ros2::eulerRpyToQuaternion(
            attitude_rpy) *
            Eigen::Vector3f{0, 0, acceleration};
          // Because we cannot compensate the gravity through the whole loop (specifically on both sides, it becomes zero),
          // we increase the compensation by a factor of 2. This is mathematically accurate, i.e., the gravity is fully
          // compensated when doing a full loop.
          float gravity_compensation_factor = 2.0f;

          // Thrust controller
          float thrust_controller_output = _thrust_controller->controllerOutput();

          if (_state == State::RotateToZero) {
            // After the initial velocity buildup, the vehicle is tilted forward, and we first let it rotate to level
            // position before entering the loop.
            if (attitude_rpy(1) > 0.0f) {
              _state = State::Powerloop;
              _thrust_controller->reset();
              RCLCPP_INFO(node().get_logger(), "Enabling thrust controller");
            } else {
              centripetal_force = 0.0f;
              gravity_compensation_factor = 1.0f;
              thrust_controller_output = 0.0f;
            }
          }

          const float gravity_compensation_force = mass * gravity_vector.dot(
            centripetal_acceleration) /
            acceleration * gravity_compensation_factor;

          const Eigen::Vector3f rate_sp{0, angular_rate, 0};
          const Eigen::Vector3f thrust_sp{0, 0,
            -centripetal_force - gravity_compensation_force - thrust_controller_output};
          _rates_setpoint->update(rate_sp, thrust_sp);

          // Allow the parameters to be changed dynamically via RC
          if (_manual_control_input->isValid()) {
            if (std::abs(_manual_control_input->roll()) > 0.05f) {
              _radius += dt_s * _manual_control_input->roll();
              _velocity = optimalVelocityFromRadius(_radius);
              RCLCPP_INFO_THROTTLE(
                node().get_logger(),
                *node().get_clock(), 200, "Radius: %.2f m, Velocity: %.2f m/s", _radius, _velocity);
            }
          }
          break;
        }
    }
  }

private:
  enum class State
  {
    BuildupVelocity,
    RotateToZero,
    Powerloop,
  };
  State _state{State::BuildupVelocity};
  std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input;
  std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_position;
  std::shared_ptr<px4_ros2::OdometryAttitude> _attitude;
  std::shared_ptr<ThrustController> _thrust_controller;

  std::shared_ptr<rclcpp::ParameterEventHandler> _param_subscriber;
  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> _cb_handles;

  // Config
  float _radius{};
  float _velocity{};
  float _start_velocity_adjustment{};
  float _hover_thrust{};
  float _attitude_delay{};
  float _thrust_p_gain{};
  float _thrust_d_gain{};
};
