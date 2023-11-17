/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <Eigen/Eigen>
#include <cmath>

// !!! move these to some math/conversions dir !!!

static inline Eigen::Vector3f quaternionToEuler(const Eigen::Quaternionf & q)
{
  auto dcm = q.toRotationMatrix();
  using Type = float;
  Eigen::Vector3f rpy;
  rpy(1) = std::asin(-dcm(2, 0));

  if ((std::fabs(rpy(1) - static_cast<Type>(M_PI / 2))) < static_cast<Type>(1.0e-3)) {
    rpy(0) = 0;
    rpy(2) = std::atan2(dcm(1, 2), dcm(0, 2));

  } else if ((std::fabs(rpy(1) + static_cast<Type>(M_PI / 2))) < static_cast<Type>(1.0e-3)) {
    rpy(0) = 0;
    rpy(2) = std::atan2(-dcm(1, 2), -dcm(0, 2));

  } else {
    rpy(0) = std::atan2(dcm(2, 1), dcm(2, 2));
    rpy(2) = std::atan2(dcm(1, 0), dcm(0, 0));
  }
  return rpy;
}

static inline float wrapPi(const float angle)
{
  const float m_pi_f = static_cast<float>(M_PI);
  if (-m_pi_f <= angle && angle < m_pi_f) {
    return angle;
  }

  const float range = 2 * m_pi_f;
  const float inv_range = 1.F / range;
  const float num_wraps = std::floor((angle + m_pi_f) * inv_range);
  return angle - range * num_wraps;
}
