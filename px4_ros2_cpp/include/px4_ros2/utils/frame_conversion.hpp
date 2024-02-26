/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

/**
 * @defgroup frame_conversion Frame Conversion
 * @ingroup utils
 * This group contains helper functions to switch between reference frames.
 */

#pragma once

#include <Eigen/Eigen>
#include <px4_ros2/utils/geometry.hpp>

namespace px4_ros2
{
/** \ingroup frame_conversion
 *  @{
 */

/**
 * @brief Converts attitude from NED to ENU frame.
 * Performs reference frame change and quaternion rotation s.t. the identity quaternion points along the x-axis within its reference frame.
 *
 * @param q_ned Attitude quaternion in NED frame.
 * @return Attitude quaternion in ENU frame.
 */
template<typename Type>
Eigen::Quaternion<Type> attitudeNedToEnu(const Eigen::Quaternion<Type> & q_ned)
{
  static constexpr Type kHalfSqrt2 = static_cast<Type>(0.7071067811865476);
  const Eigen::Quaternion<Type> q_ned_to_enu {0.0, kHalfSqrt2, kHalfSqrt2, 0.0};  // 180 deg along X, -90 deg along Z (intrinsic)
  const Eigen::Quaternion<Type> q_yaw_minus_pi_2{kHalfSqrt2, 0.0, 0.0, -kHalfSqrt2};  // -90 deg along Z

  return q_ned_to_enu * q_ned * q_yaw_minus_pi_2 * q_ned_to_enu.inverse();
}

/**
 * @brief Converts attitude from ENU to NED frame.
 * Performs reference frame change and quaternion rotation s.t. the identity quaternion points along the x-axis within its reference frame.
 *
 * @param q_enu Attitude quaternion in ENU frame.
 * @return Attitude quaternion in NED frame.
 */
template<typename Type>
Eigen::Quaternion<Type> attitudeEnuToNed(const Eigen::Quaternion<Type> & q_enu)
{
  static constexpr Type kHalfSqrt2 = static_cast<Type>(0.7071067811865476);
  const Eigen::Quaternion<Type> q_enu_to_ned {0.0, kHalfSqrt2, kHalfSqrt2, 0.0};  // 180 deg along X, -90 deg along Z (intrinsic)
  const Eigen::Quaternion<Type> q_yaw_minus_pi_2{kHalfSqrt2, 0.0, 0.0, -kHalfSqrt2};  // -90 deg along Z

  return q_enu_to_ned * q_enu * q_yaw_minus_pi_2 * q_enu_to_ned.inverse();
}

/**
 * @brief Transforms a point from body frame to world frame given the yaw of the body's attitude.
 *
 * @param yaw The yaw of the body's attitude [rad].
 * @param point_body The point coordinates in the body frame.
 * @return The yaw rotated point in the world frame.
 */
template<typename Type>
Eigen::Matrix<Type, 3, 1> yawBodyToWorld(
  Type yaw,
  const Eigen::Matrix<Type, 3, 1> & point_body)
{
  const Eigen::Quaternion<Type> q_z(Eigen::AngleAxis<Type>(
      yaw, Eigen::Matrix<Type, 3, 1>::UnitZ()));
  return q_z * point_body;
}

/**
 * @brief Converts yaw from NED to ENU frame.
 *
 * @param yaw_ned_rad Yaw angle in NED frame [rad].
 * @return Yaw angle in ENU frame [rad], wrapped to [-pi, pi].
 */
template<typename T>
static inline T yawNedToEnu(const T yaw_ned_rad)
{
  return wrapPi(static_cast<T>(M_PI / 2.0) - yaw_ned_rad);
}

/**
 * @brief Converts yaw from ENU to NED frame.
 *
 * @param yaw_enu_rad Yaw angle in ENU frame [rad].
 * @return Yaw angle in NED frame [rad], wrapped to [-pi, pi].
 */
template<typename T>
static inline T yawEnuToNed(const T yaw_enu_rad)
{
  return wrapPi(static_cast<T>(M_PI / 2.0) - yaw_enu_rad);
}

/**
 * @brief Converts yaw rate from NED to ENU frame.
 *
 * @param yaw_rate_ned Yaw rate in NED frame.
 * @return Yaw rate in ENU frame.
 */
template<typename T>
static inline T yawRateNedToEnu(const T yaw_rate_ned) {return -yaw_rate_ned;}

/**
 * @brief Converts yaw rate from ENU to NED frame.
 *
 * @param yaw_rate_enu Yaw rate in ENU frame.
 * @return Yaw rate in NED frame.
 */
template<typename T>
static inline T yawRateEnuToNed(const T yaw_rate_enu) {return -yaw_rate_enu;}

/**
 * @brief Converts coordinates from NED to ENU frame.
 *
 * @param ned Coordinates in NED frame.
 * @return Coordinates in ENU frame.
 */
template<typename T>
static inline Eigen::Matrix<T, 3, 1> positionNedToEnu(const Eigen::Matrix<T, 3, 1> & ned)
{
  return {ned.y(), ned.x(), -ned.z()};
}

/**
 * @brief Converts coordinates from ENU to NED frame.
 *
 * @param enu Coordinates in ENU frame.
 * @return Coordinates in NED frame.
 */
template<typename T>
static inline Eigen::Matrix<T, 3, 1> positionEnuToNed(const Eigen::Matrix<T, 3, 1> & enu)
{
  return {enu.y(), enu.x(), -enu.z()};
}

/**
 * @brief Converts coordinates from FRD to FLU frame.
 *
 * @param frd Coordinates in FRD frame.
 * @return Coordinates in FLU frame.
 */
template<typename T>
static inline Eigen::Matrix<T, 3, 1> frdToFlu(const Eigen::Matrix<T, 3, 1> & frd)
{
  return {frd.x(), -frd.y(), -frd.z()};
}

/**
 * @brief Converts coordinates from FLU to FRD frame.
 *
 * @param flu Coordinates in FLU frame.
 * @return Coordinates in FRD frame.
 */
template<typename T>
static inline Eigen::Matrix<T, 3, 1> fluToFrd(const Eigen::Matrix<T, 3, 1> & flu)
{
  return {flu.x(), -flu.y(), -flu.z()};
}

/**
 * @brief Converts variance from NED to ENU frame.
 *
 * @param v_ned Variance vector in NED frame.
 * @return Variance vector in ENU frame.
 */
template<typename T>
static inline Eigen::Matrix<T, 3, 1> varianceNedToEnu(const Eigen::Matrix<T, 3, 1> & v_ned)
{
  return {v_ned.y(), v_ned.x(), v_ned.z()};
}

/**
 * @brief Converts variance from ENU to NED frame.
 *
 * @param v_enu Variance vector in ENU frame.
 * @return Variance vector in NED frame.
 */
template<typename T>
static inline Eigen::Matrix<T, 3, 1> varianceEnuToNed(const Eigen::Matrix<T, 3, 1> & v_enu)
{
  return {v_enu.y(), v_enu.x(), v_enu.z()};
}

/** @}*/
}  // namespace px4_ros2
