/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

/*
 * Azimuthal Equidistant Projection
 * formulas according to: http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html
 */

#include "map_projection_impl.hpp"


namespace px4_ros2
{

void MapProjectionImpl::initReference(double lat_0, double lon_0, double alt_0, uint64_t timestamp)
{
  _ref_timestamp = timestamp;
  _ref_lat = degToRad(lat_0);
  _ref_lon = degToRad(lon_0);
  _ref_alt_amsl = alt_0;
  _ref_sin_lat = std::sin(_ref_lat);
  _ref_cos_lat = std::cos(_ref_lat);
  _ref_init_done = true;
}

Eigen::Vector2f MapProjectionImpl::globalToLocal(const Eigen::Vector2d & global_position) const
{
  Eigen::Vector2f local_position;

  const double lat_rad = degToRad(global_position.x());
  const double lon_rad = degToRad(global_position.y());

  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);

  const double cos_d_lon = std::cos(lon_rad - _ref_lon);

  const double arg = std::clamp(
    _ref_sin_lat * sin_lat + _ref_cos_lat * cos_lat * cos_d_lon, -1.0,
    1.0);
  const double c = std::acos(arg);

  double k = 1.0;

  if (std::fabs(c) > 0) {
    k = (c / std::sin(c));
  }

  local_position.x() =
    static_cast<float>(k * (_ref_cos_lat * sin_lat - _ref_sin_lat * cos_lat * cos_d_lon) *
    kRadiusOfEarth);
  local_position.y() =
    static_cast<float>(k * cos_lat * std::sin(lon_rad - _ref_lon) * kRadiusOfEarth);

  return local_position;
}

Eigen::Vector2d MapProjectionImpl::localToGlobal(const Eigen::Vector2f & local_position) const
{
  Eigen::Vector2d global_position;

  const double x_rad = static_cast<double>(local_position.x()) / kRadiusOfEarth;
  const double y_rad = static_cast<double>(local_position.y()) / kRadiusOfEarth;
  const double c = std::sqrt(x_rad * x_rad + y_rad * y_rad);

  if (std::fabs(c) > 0) {
    const double sin_c = std::sin(c);
    const double cos_c = std::cos(c);

    const double lat_rad = std::asin(cos_c * _ref_sin_lat + (x_rad * sin_c * _ref_cos_lat) / c);
    const double lon_rad =
      (_ref_lon +
      std::atan2(y_rad * sin_c, c * _ref_cos_lat * cos_c - x_rad * _ref_sin_lat * sin_c));

    global_position.x() = radToDeg(lat_rad);
    global_position.y() = radToDeg(lon_rad);

  } else {
    global_position.x() = radToDeg(_ref_lat);
    global_position.y() = radToDeg(_ref_lon);
  }

  return global_position;
}

} // namespace px4_ros2
