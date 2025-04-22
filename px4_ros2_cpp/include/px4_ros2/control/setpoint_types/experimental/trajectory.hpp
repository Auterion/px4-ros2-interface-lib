/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <Eigen/Eigen>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types_experimental
 *  @{
 */

struct TrajectorySetpoint;

/**
 * @brief Setpoint type for trajectory control
 *
 * Control entries must not be contradicting.
*/
class TrajectorySetpointType : public SetpointBase
{
public:
  /**
   * setting local_position_is_optional to true allows to create a mode that uses trajectory
   * setpoint but doesn't necessarly require local position. Sending XY position setpoint
   * without a correct source of positional data is not recommended
   */
  explicit TrajectorySetpointType(Context & context, bool local_position_is_optional = false);

  ~TrajectorySetpointType() override = default;

  Configuration getConfiguration() override;

  void update(
    const Eigen::Vector3f & velocity_ned_m_s,
    const std::optional<Eigen::Vector3f> & acceleration_ned_m_s2 = {},
    std::optional<float> yaw_ned_rad = {},
    std::optional<float> yaw_rate_ned_rad_s = {});

  /**
   * @brief Update the setpoint with full flexibility by passing a TrajectorySetpoint
   *
   * @param setpoint a TrajectorySetpoint object where position, velocity and acceleration
   * can be set individually for full flexibility
   *
   * @warning This method is not recommended for general use because it is possible
   * to set contradicting setpoint values that can result in an unstable system
   * and to crashes.
   */
  void update(const TrajectorySetpoint & setpoint);

  /**
   * @brief Position setpoint update.
   *
   * The GotoSetpointType should be preferred.
   *
   * @param position_ned_m [m] NED earth-fixed frame
   */
  void updatePosition(
    const Eigen::Vector3f & position_ned_m);

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_setpoint_pub;

  const bool _local_position_is_optional;
};

/** @}*/


/**
 * @brief Setpoint structure for trajectory control with fine-grained control over individual components
 *
 * This structure allows setting position, velocity, acceleration, yaw, and yaw rate
 * individually on a per-axis basis. All fields are optional which means unset fields will
 * not be controlled (NaN values will be sent to PX4).
 *
 * @warning Combining certain types of setpoints (e.g., position and velocity on the same axis)
 * can lead to inconsistencies and unstable behavior. Use with caution.
 *
 * @note For simple position control, use TrajectorySetpointType::updatePosition() instead, or even
 * better, use the GotoSetpointTypeif possible.
 *
 * ## Example: Altitude hold with horizontal velocity
 * ```cpp
 * TrajectorySetpoint setpoint;
 * setpoint.withHorizontalVelocity(Eigen::Vector2f(1.0f, 0.5f)) // Move at 1 m/s forward, 0.5 m/s right
 *         .withPositionZ(-2.0f);                               // Hold altitude at 2 meters above ground
 * trajectory_setpoint_type->update(setpoint);
 * ```
 */
struct TrajectorySetpoint
{
  std::optional<float> position_ned_m_x;
  std::optional<float> position_ned_m_y;
  std::optional<float> position_ned_m_z;

  std::optional<float> velocity_ned_m_s_x;
  std::optional<float> velocity_ned_m_s_y;
  std::optional<float> velocity_ned_m_s_z;

  std::optional<float> acceleration_ned_m_s2_x;
  std::optional<float> acceleration_ned_m_s2_y;
  std::optional<float> acceleration_ned_m_s2_z;

  std::optional<float> yaw_ned_rad;
  std::optional<float> yaw_rate_ned_rad_s;


  /**
   * @brief Set position setpoint for x-axis in NED frame
   * @param x_ned_m Position in x-axis [m]
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withPositionX(float x_ned_m)
  {
    position_ned_m_x = x_ned_m;
    return *this;
  }

  /**
   * @brief Set position setpoint for y-axis in NED frame
   * @param y_ned_m Position in y-axis [m]
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withPositionY(float y_ned_m)
  {
    position_ned_m_y = y_ned_m;
    return *this;
  }

  /**
   * @brief Set position setpoint for z-axis in NED frame
   * @param z_ned_m Position in z-axis [m] (negative = above ground in NED frame)
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withPositionZ(float z_ned_m)
  {
    position_ned_m_z = z_ned_m;
    return *this;
  }

  /**
   * @brief Set velocity setpoint for x-axis in NED frame
   * @param x_ned_m_s Velocity in x-axis [m/s]
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withVelocityX(float x_ned_m_s)
  {
    velocity_ned_m_s_x = x_ned_m_s;
    return *this;
  }

  /**
   * @brief Set velocity setpoint for y-axis in NED frame
   * @param y_ned_m_s Velocity in y-axis [m/s]
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withVelocityY(float y_ned_m_s)
  {
    velocity_ned_m_s_y = y_ned_m_s;
    return *this;
  }

  /**
   * @brief Set velocity setpoint for z-axis in NED frame
   * @param z_ned_m_s Velocity in z-axis [m/s] (negative = upward in NED frame)
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withVelocityZ(float z_ned_m_s)
  {
    velocity_ned_m_s_z = z_ned_m_s;
    return *this;
  }

  /**
   * @brief Set acceleration setpoint for x-axis in NED frame
   * @param x_ned_m_s2 Acceleration in x-axis [m/s²]
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withAccelerationX(float x_ned_m_s2)
  {
    acceleration_ned_m_s2_x = x_ned_m_s2;
    return *this;
  }

  /**
   * @brief Set acceleration setpoint for y-axis in NED frame
   * @param y_ned_m_s2 Acceleration in y-axis [m/s²]
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withAccelerationY(float y_ned_m_s2)
  {
    acceleration_ned_m_s2_y = y_ned_m_s2;
    return *this;
  }

  /**
   * @brief Set acceleration setpoint for z-axis in NED frame
   * @param z_ned_m_s2 Acceleration in z-axis [m/s²] (negative = upward in NED frame)
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withAccelerationZ(float z_ned_m_s2)
  {
    acceleration_ned_m_s2_z = z_ned_m_s2;
    return *this;
  }

  /**
   * @brief Set yaw setpoint in NED frame
   * @param yaw_rad Yaw angle [rad] (0 = North, PI/2 = East)
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withYaw(float yaw_rad)
  {
    this->yaw_ned_rad = yaw_rad;
    return *this;
  }

  /**
   * @brief Set yaw rate setpoint in NED frame
   * @param rate_rad_s Yaw angular velocity [rad/s]
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withYawRate(float rate_rad_s)
  {
    this->yaw_rate_ned_rad_s = rate_rad_s;
    return *this;
  }

  // Helper methods for setting multiple components at once
  /**
   * @brief Set position setpoint for all axes (x, y, z) in NED frame
   * @param position_ned_m Position vector in NED frame [m]
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withPosition(const Eigen::Vector3f & position_ned_m)
  {
    position_ned_m_x = position_ned_m.x();
    position_ned_m_y = position_ned_m.y();
    position_ned_m_z = position_ned_m.z();
    return *this;
  }

  /**
   * @brief Set position setpoint for horizontal axes (x, y) in NED frame
   * @param position_ned_m Position vector in NE plane [m]
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withHorizontalPosition(const Eigen::Vector2f & position_ned_m)
  {
    position_ned_m_x = position_ned_m.x();
    position_ned_m_y = position_ned_m.y();
    return *this;
  }

  /**
   * @brief Set velocity setpoint for all axes (x, y, z) in NED frame
   * @param velocity_ned_m_s Velocity vector in NED frame [m/s]
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withVelocity(const Eigen::Vector3f & velocity_ned_m_s)
  {
    velocity_ned_m_s_x = velocity_ned_m_s.x();
    velocity_ned_m_s_y = velocity_ned_m_s.y();
    velocity_ned_m_s_z = velocity_ned_m_s.z();
    return *this;
  }

  /**
   * @brief Set velocity setpoint for horizontal axes (x, y) in NED frame
   * @param velocity_ned_m_s Velocity vector in NE plane [m/s]
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withHorizontalVelocity(const Eigen::Vector2f & velocity_ned_m_s)
  {
    velocity_ned_m_s_x = velocity_ned_m_s.x();
    velocity_ned_m_s_y = velocity_ned_m_s.y();
    return *this;
  }

  /**
   * @brief Set acceleration setpoint for all axes (x, y, z) in NED frame
   * @param acceleration_ned_m_s2 Acceleration vector in NED frame [m/s²]
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withAcceleration(const Eigen::Vector3f & acceleration_ned_m_s2)
  {
    acceleration_ned_m_s2_x = acceleration_ned_m_s2.x();
    acceleration_ned_m_s2_y = acceleration_ned_m_s2.y();
    acceleration_ned_m_s2_z = acceleration_ned_m_s2.z();
    return *this;
  }

  /**
   * @brief Set acceleration setpoint for horizontal axes (x, y) in NED frame
   * @param acceleration_ned_m_s2 Acceleration vector in NE plane [m/s²]
   * @return Reference to self for method chaining
   */
  TrajectorySetpoint & withHorizontalAcceleration(const Eigen::Vector2f & acceleration_ned_m_s2)
  {
    acceleration_ned_m_s2_x = acceleration_ned_m_s2.x();
    acceleration_ned_m_s2_y = acceleration_ned_m_s2.y();
    return *this;
  }
};


} /* namespace px4_ros2 */
