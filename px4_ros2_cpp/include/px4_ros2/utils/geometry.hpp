/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>

namespace px4_ros2
{

/**
 * @brief Wraps an angle to the range [-pi, pi).
 *
 * @param angle The input angle [rad].
 * @return The wrapped angle in the range [-pi, pi).
 */
template<typename Type>
Type wrapPi(Type angle)
{
  while (angle >= M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

/**
 * @brief Converts a quaternion to Euler angles.
 *
 * @param q The input quaternion.
 * @return Euler angles (order: RPY) corresponding to the given quaternion in range r, p, y = [-pi, pi], [-pi/2, pi], [-pi, pi].
 */
template<typename Type>
Eigen::Matrix<Type, 3, 1> quaternionToEulerRPY(const Eigen::Quaternion<Type> & q)
{
  Eigen::Matrix<Type, 3, 1> angles;
  Eigen::Matrix<Type, 3, 3> dcm = q.toRotationMatrix();

  angles.y() = asin(-dcm.coeff(2, 0));

  if ((std::fabs(angles.y() - static_cast<Type>(M_PI / 2))) < static_cast<Type>(1.0e-3)) {
    angles.x() = 0;
    angles.z() = atan2(dcm.coeff(1, 2), dcm.coeff(0, 2));
  } else if ((std::fabs(angles.y() + static_cast<Type>(M_PI / 2))) < static_cast<Type>(1.0e-3)) {
    angles.x() = 0;
    angles.z() = atan2(-dcm.coeff(1, 2), -dcm.coeff(0, 2));
  } else {
    angles.x() = atan2(dcm.coeff(2, 1), dcm.coeff(2, 2));
    angles.z() = atan2(dcm.coeff(1, 0), dcm.coeff(0, 0));
  }

  return angles;
}

/**
 * @brief Convert quaternion to roll angle;
 *
 * @param q The input quaternion
 * @return Roll angle (order: RPY) corresponding to the given quaternion in range [-pi, pi]
*/
template<typename Type>
Type quaternionToRoll(const Eigen::Quaternion<Type> & q)
{
  Type x = q.x();
  Type y = q.y();
  Type z = q.z();
  Type w = q.w();

  return std::atan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z);
}

/**
 * @brief Convert quaternion to pitch angle;
 *
 * @param q The input quaternion
 * @return Pitch angle (order: RPY) corresponding to the given quaternion in range [-pi, pi]
*/
template<typename Type>
Type quaternionToPitch(const Eigen::Quaternion<Type> & q)
{
  Type x = q.x();
  Type y = q.y();
  Type z = q.z();
  Type w = q.w();

  return std::atan2(2.0 * (w * y - z * x), 1.0 - 2.0 * (x * x + y * y));
}

/**
 * @brief Convert quaternion to yaw angle;
 *
 * @param q The input quaternion
 * @return Yaw angle (order: RPY) corresponding to the given quaternion in range [-pi, pi]
*/
template<typename Type>
Type quaternionToYaw(const Eigen::Quaternion<Type> & q)
{
  Type x = q.x();
  Type y = q.y();
  Type z = q.z();
  Type w = q.w();

  return std::atan2(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z);
}

}  // namespace px4_ros2
