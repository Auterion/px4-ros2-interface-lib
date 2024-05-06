/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <cmath>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Eigen>

namespace px4_ros2
{

static constexpr double kRadiusOfEarth = 6371000;                                    // meters (m)

/**
 * @brief C++ class for mapping lat/lon coordinates to local coordinates using a reference position
 */
class MapProjectionImpl final
{
private:
  uint64_t _ref_timestamp{0};       /**< Reference time (microseconds), since system start */
  double _ref_lat{0.0};             /**< Reference point latitude (radians) */
  double _ref_lon{0.0};             /**< Reference point longitude (radians) */
  double _ref_alt_amsl{0.0};        /**< Reference point altitude AMSL (meters) */
  double _ref_sin_lat{0.0};
  double _ref_cos_lat{0.0};
  bool _ref_init_done{false};

public:
  /**
   * @brief Construct a new Map Projection object
   * The generated object will be uninitialized.
   * To initialize, use the `initReference` function
   */
  explicit MapProjectionImpl() = default;

  /**
   * @brief Construct and initialize a new Map Projection object
   */
  explicit MapProjectionImpl(double lat_0, double lon_0, double alt_0, uint64_t timestamp)
  {
    initReference(lat_0, lon_0, alt_0, timestamp);
  }

  /**
   * @brief Initialize the map transformation
   *
   * with reference coordinates on the geographic coordinate system
   * where the azimuthal equidistant plane's origin is located
   * @param lat in degrees (47.1234567°, not 471234567°)
   * @param lon in degrees (8.1234567°, not 81234567°)
   */
  void initReference(double lat_0, double lon_0, double alt_0, uint64_t timestamp);

  /**
   * @return true, if the map reference has been initialized before
   */
  inline bool isInitialized() const {return _ref_init_done;}

  /**
   * @return the timestamp of the reference which the map projection was initialized with
   */
  inline uint64_t getProjectionReferenceTimestamp() const {return _ref_timestamp;}

  /**
   * @return the projection reference latitude in degrees
   */
  inline double getProjectionReferenceLat() const {return radToDeg(_ref_lat);}

  /**
   * @return the projection reference longitude in degrees
   */
  inline double getProjectionReferenceLon() const {return radToDeg(_ref_lon);}

  /**
   * @return the projection reference altitude AMSL in meters
   */
  inline double getProjectionReferenceAlt() const {return _ref_alt_amsl;}

  /**
   * @return the projection reference position: lat [deg], lon [deg], alt AMSL [m]
   */
  inline Eigen::Vector3d getProjectionReferencePosition() const
  {
    return Eigen::Vector3d{radToDeg(_ref_lat), radToDeg(_ref_lon), _ref_alt_amsl};
  }

  /**
   * Transform a point in the geographic coordinate system to the local
   * azimuthal equidistant plane using the projection
   *
   * @param global_position lat [deg], lon [deg]
   * @return the point in local coordinates as north, east [m]
   */
  Eigen::Vector2f globalToLocal(const Eigen::Vector2d & global_position) const;

  /**
   * Transform a point in the geographic coordinate system to the local
   * azimuthal equidistant plane using the projection
   *
   * @param global_position lat [deg], lon [deg], alt AMSL [m]
   * @return the point in local coordinates as north, east, down [m]
   */
  inline Eigen::Vector3f globalToLocal(const Eigen::Vector3d & global_position) const
  {
    Eigen::Vector3f local_position;

    local_position.template head<2>() =
      globalToLocal(static_cast<Eigen::Vector2d>(global_position.head(2)));
    local_position.z() = static_cast<float>(_ref_alt_amsl - global_position.z());
    return local_position;
  }

  /**
   * Transform a point in the local azimuthal equidistant plane to the
   * geographic coordinate system using the projection
   *
   * @param local_position north, east [m]
   * @return the point in geographic coordinates as lat [deg], lon [deg]
   */
  Eigen::Vector2d localToGlobal(const Eigen::Vector2f & local_position) const;

  /**
   * Transform a point in the local azimuthal equidistant plane to the
   * geographic coordinate system using the projection
   *
   * @param local_position north, east, down [m]
   * @return the point in geographic coordinates as lat [deg], lon [deg], alt AMSL [m]
   */
  inline Eigen::Vector3d localToGlobal(const Eigen::Vector3f & local_position) const
  {
    Eigen::Vector3d global_position;

    global_position.template head<2>() =
      localToGlobal(static_cast<Eigen::Vector2f>(local_position.head(2)));
    global_position.z() = static_cast<double>(_ref_alt_amsl - local_position.z());
    return global_position;
  }
};

}  // namespace px4_ros2
