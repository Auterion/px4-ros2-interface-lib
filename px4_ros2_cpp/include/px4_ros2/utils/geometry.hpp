/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

/**
 * @defgroup geometry Geometry
 * @ingroup utils
 *
 * All rotations and axis systems follow the right-hand rule
 *
 * Euler angles follow the convention of a 1-2-3 extrinsic Tait-Bryan rotation sequence.
 * In order to go from frame 1 to frame 2 we apply the following rotations consecutively.
 * 1) We rotate about our fixed X axis by an angle of "roll"
 * 2) We rotate about our fixed Y axis by an angle of "pitch"
 * 3) We rotate abour our fixed Z axis by an angle of "yaw"
 *
 * Note that this convention is equivalent to that of a
 * 3-2-1 intrinsic Tait-Bryan rotation sequence, i.e.
 * 1) We rotate about our initial Z axis by an angle of "yaw"
 * 2) We rotate about the newly created Y' axis by an angle of "pitch"
 * 3) We rotate about the newly created X'' axis by an angle of "roll"
 */

#pragma once

#include <Eigen/Eigen>

namespace px4_ros2
{
/** \ingroup geometry
 *  @{
 */

/**
 * @brief Converts radians to degrees
 */
template<typename Type>
Type radToDeg(Type rad)
{
  return rad * static_cast<Type>(180.0 / M_PI);
}

/**
 * @brief Converts degrees to radians
 */
template<typename Type>
Type degToRad(Type deg)
{
  return deg * static_cast<Type>(M_PI / 180.0);
}

namespace literals
{
static inline constexpr float operator"" _deg(long double degrees)
{
  return static_cast<float>(degrees * M_PI / 180.0);
}
static inline constexpr float operator"" _rad(long double radians)
{
  return static_cast<float>(radians);
}
}

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
 * @brief Converts a quaternion to RPY extrinsic Tait-Bryan Euler angles (YPR intrinsic)
 * XYZ axes correspond to RPY angles respectively.
 *
 * @param q The input quaternion.
 * @return Euler angles corresponding to the given quaternion in range R, P, Y = [-pi, pi], [-pi/2, pi/2], [-pi, pi].
 */
template<typename Type>
Eigen::Matrix<Type, 3, 1> quaternionToEulerRpy(const Eigen::Quaternion<Type> & q)
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
 * @brief Converts RPY extrinsic Tait-Bryan Euler angles (YPR intrinsic) to quaternion
 *
 * @param euler The euler angles [rad]
 * @return Quaternion corresponding to the given euler angles.
 */
template<typename Type>
Eigen::Quaternion<Type> eulerRpyToQuaternion(const Eigen::Matrix<Type, 3, 1> & euler)
{
  return Eigen::Quaternion<Type>(
    Eigen::AngleAxis<Type>(euler[2], Eigen::Matrix<Type, 3, 1>::UnitZ()) *
    Eigen::AngleAxis<Type>(euler[1], Eigen::Matrix<Type, 3, 1>::UnitY()) *
    Eigen::AngleAxis<Type>(euler[0], Eigen::Matrix<Type, 3, 1>::UnitX()));
}

/**
 * @brief Converts RPY extrinsic Tait-Bryan Euler angles (YPR intrinsic) to quaternion
 *
 * @param roll The roll angle [rad].
 * @param pitch The pitch angle [rad].
 * @param yaw The yaw angle [rad].
 * @return Quaternion corresponding to the given euler angles.
 */
template<typename Type>
Eigen::Quaternion<Type> eulerRpyToQuaternion(const Type roll, const Type pitch, const Type yaw)
{
  return eulerRpyToQuaternion(Eigen::Matrix<Type, 3, 1>{roll, pitch, yaw});
}

/**
 * @brief Convert quaternion to roll angle in extrinsic RPY order (intrinsic YPR)
 *
 * @param q The input quaternion
 * @return Roll angle corresponding to the given quaternion in range [-pi, pi]
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
 * @brief Convert quaternion to pitch angle in extrinsic RPY order (intrinsic YPR)
 *
 * @param q The input quaternion
 * @return Pitch angle corresponding to the given quaternion in range [-pi, pi]
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
 * @brief Convert quaternion to yaw angle in extrinsic RPY order (intrinsic YPR)
 *
 * @param q The input quaternion
 * @return Yaw angle corresponding to the given quaternion in range [-pi, pi]
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

/** @}*/
}  // namespace px4_ros2
