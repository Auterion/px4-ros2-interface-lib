/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

/**
 * @defgroup geodesic Geodesic
 * @ingroup utils
 *
 * This group contains helper functions to convert points between
 * the geographical coordinate system ("global") and the local azimuthal
 * equidistant plane ("local"). Additionaly it provides methods for commonly
 * used geodesic operations.
 *
 * Latitude and longitude are in degrees (degrees: 8.1234567°, not 81234567°).
 * Altitude is in meters in the above mean sea level (AMSL) frame.
 * Heading is in radians starting North going clockwise.
 */

#pragma once

#include <cmath>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_ros2/common/context.hpp>

namespace px4_ros2
{
/** \ingroup geodesic
 *  @{
 */

class MapProjectionImpl;

/**
 * @brief Provides methods to convert between the geographical coordinate system ("global")
 * and the local azimuthal equidistant plane ("local").
 *
 * This class handles ROS2 subscription and initializing the map projection reference point from PX4.
 * The mathematical implementation is contained in `px4_ros2::MapProjectionImpl`.
 *
 * @see px4_ros2::MapProjectionImpl
 * @ingroup geodesic
 */
class MapProjection
{
public:
  explicit MapProjection(Context & context);

  ~MapProjection();

  /**
   * @return true, if the map reference has been initialized before
   */
  bool isInitialized() const;

  /**
   * @brief Transform a point in the geographic coordinate system to the local
   * azimuthal equidistant plane using the projection
   *
   * @param global_position lat [deg], lon [deg]
   * @return the point in local coordinates as north, east [m]
   */
  Eigen::Vector2f globalToLocal(const Eigen::Vector2d & global_position) const;

  /**
   * @brief Transform a point in the geographic coordinate system to the local
   * azimuthal equidistant plane using the projection
   *
   * @param global_position lat [deg], lon [deg], alt AMSL [m]
   * @return the point in local coordinates as north, east, down [m]
   */
  Eigen::Vector3f globalToLocal(const Eigen::Vector3d & global_position) const;

  /**
   * @brief Transform a point in the local azimuthal equidistant plane to the
   * geographic coordinate system using the projection
   *
   * @param local_position north, east [m]
   * @return the point in geographic coordinates as lat [deg], lon [deg]
   */
  Eigen::Vector2d localToGlobal(const Eigen::Vector2f & local_position) const;

  /**
   * @brief Transform a point in the local azimuthal equidistant plane to the
   * geographic coordinate system using the projection
   *
   * @param local_position north, east, down [m]
   * @return the point in geographic coordinates as lat [deg], lon [deg], alt AMSL [m]
   */
  Eigen::Vector3d localToGlobal(const Eigen::Vector3f & local_position) const;

private:
  /**
   * @throw runtime error if class instance is not initalized
   */
  void assertInitalized() const;

  /**
   * @brief Callback for VehicleLocalPosition messages which intializes and updates the map projection reference point from PX4
   *
   * @param msg the VehicleLocalPosition message
  */
  void vehicleLocalPositionCallback(px4_msgs::msg::VehicleLocalPosition::UniquePtr msg);

  rclcpp::Node & _node;
  std::unique_ptr<MapProjectionImpl> _map_projection_math;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _vehicle_local_position_sub;
};

/**
 * @brief Compute the horizontal distance between two global positions in meters.
 *
 * @param global_position_now current lat [deg], lon [deg]
 * @param global_position_next next lat [deg], lon [deg]
 * @return the horizontal distance [m] between both positions
 *
 * @ingroup geodesic
 */
float horizontalDistanceToGlobalPosition(
  const Eigen::Vector2d & global_position_now,
  const Eigen::Vector2d & global_position_next);

/**
 * @brief Compute the horizontal distance between two global positions in meters.
 *
 * @param global_position_now current lat [deg], lon [deg], alt AMSL [m]
 * @param global_position_next next lat [deg], lon [deg], alt AMSL [m]
 * @return the horizontal distance [m] between both positions

 * @ingroup geodesic
 */
static inline float horizontalDistanceToGlobalPosition(
  const Eigen::Vector3d & global_position_now,
  const Eigen::Vector3d & global_position_next)
{
  return horizontalDistanceToGlobalPosition(
    static_cast<Eigen::Vector2d>(global_position_now.head(2)),
    static_cast<Eigen::Vector2d>(global_position_next.head(2)));
}

/**
 * @brief Compute the distance between two global positions in meters.
 *
 * @param global_position_now current lat [deg], lon [deg]
 * @param global_position_next next lat [deg], lon [deg]
 * @return the distance [m] between both positions

 * @ingroup geodesic
 */
float distanceToGlobalPosition(
  const Eigen::Vector3d & global_position_now,
  const Eigen::Vector3d & global_position_next);

/**
 * @brief Compute the heading to the next global position in radians.
 *
 * @param global_position_now current lat [deg], lon [deg]
 * @param global_position_next next lat [deg], lon [deg]
 * @return the heading [rad] to the next global position (clockwise)
 *
 * @ingroup geodesic
*/
float headingToGlobalPosition(
  const Eigen::Vector2d & global_position_now,
  const Eigen::Vector2d & global_position_next);

/**
 * @brief Compute the heading to the next global position in radians.
 *
 * @param global_position_now current lat [deg], lon [deg], alt AMSL [m]
 * @param global_position_next next lat [deg], lon [deg], alt AMSL [m]
 * @return the heading [rad] to the next global position (clockwise)
 *
 * @ingroup geodesic
*/
static inline float headingToGlobalPosition(
  const Eigen::Vector3d & global_position_now,
  const Eigen::Vector3d & global_position_next)
{
  return headingToGlobalPosition(
    static_cast<Eigen::Vector2d>(global_position_now.head(2)),
    static_cast<Eigen::Vector2d>(global_position_next.head(2)));
}

/**
 * @brief Compute the vector to the next global position in meters.
 *
 * @param global_position_now current lat [deg], lon [deg]
 * @param global_position_next next lat [deg], lon [deg]
 * @return the vector [m^2] in local frame to the next global position (NE)
 *
 * @ingroup geodesic
*/
Eigen::Vector2f vectorToGlobalPosition(
  const Eigen::Vector2d & global_position_now,
  const Eigen::Vector2d & global_position_next);

/**
 * @brief Compute the vector to the next global position in meters.
 *
 * @param global_position_now current lat [deg], lon [deg], alt AMSL [m]
 * @param global_position_next next lat [deg], lon [deg], alt AMSL [m]
 * @return the vector [m^3] in local frame to the next global position (NED)
 *
 * @ingroup geodesic
*/
static inline Eigen::Vector3f vectorToGlobalPosition(
  const Eigen::Vector3d & global_position_now,
  const Eigen::Vector3d & global_position_next)
{
  const Eigen::Vector2f vector_res =
    vectorToGlobalPosition(
    static_cast<Eigen::Vector2d>(global_position_now.head(2)),
    static_cast<Eigen::Vector2d>(global_position_next.head(2)));
  const double d_alt = -(global_position_next.z() - global_position_now.z());
  return Eigen::Vector3f {vector_res.x(), vector_res.y(), static_cast<float>(d_alt)};
}

/**
 * @brief Compute the global position on the line vector defined by two positions (start -> end) at a certain distance
 * from global position 'start'.
 *
 * @param global_position_line_start line start lat [deg], lon [deg]
 * @param global_position_line_end line end lat [deg], lon [deg]
 * @param dist_from_start distance [m] of target global position from position 'start' towards position 'end' (can be negative)
 *
 * @ingroup geodesic
 */
Eigen::Vector2d globalPositionFromLineAndDist(
  const Eigen::Vector2d & global_position_line_start,
  const Eigen::Vector2d & global_position_line_end,
  float dist_from_start);

/**
 * @brief Compute the global position given another global position, distance and heading
 * see http://www.movable-type.co.uk/scripts/latlong.html
 *
 * @param global_position_now current lat [deg], lon [deg]
 * @param heading heading from the current position [rad] (clockwise)
 * @param dist distance from the current position [m]
 * @return the target global position
 *
 * @ingroup geodesic
 */
Eigen::Vector2d globalPositionFromHeadingAndDist(
  const Eigen::Vector2d & global_position_now,
  float heading, float dist);

/**
 * @brief Compute the global position given another global position, distance and heading
 * see http://www.movable-type.co.uk/scripts/latlong.html
 *
 * @param global_position_now current lat [deg], lon [deg]
 * @param heading heading from the current position [rad] (clockwise)
 * @param dist distance from the current position [m]
 * @return the target global position
 *
 * @ingroup geodesic
 */
static inline Eigen::Vector3d globalPositionFromHeadingAndDist(
  const Eigen::Vector3d & global_position_now,
  float heading, float dist)
{
  const Eigen::Vector2d global_position_res = globalPositionFromHeadingAndDist(
    static_cast<Eigen::Vector2d>(global_position_now.head(2)), heading, dist);
  return Eigen::Vector3d {
    global_position_res.x(),
    global_position_res.y(),
    global_position_now.z()};
}

/**
 * @brief Compute the global position from adding a local frame vector to the current global position.
 *
 * @param global_position current lat [deg], lon [deg]
 * @param vector_ne local vector to add [m^2] (NE)
 * @return the resulting global position from the addition
 *
 * @ingroup geodesic
*/
Eigen::Vector2d addVectorToGlobalPosition(
  const Eigen::Vector2d & global_position,
  const Eigen::Vector2f & vector_ne);

/**
 * @brief Compute the global position from adding a local frame vector to the current global position.
 *
 * @param global_position current lat [deg], lon [deg], alt AMSL [m]
 * @param vector_ned local vector to add [m^3] (NED)
 * @return the resulting global position from the addition
 *
 * @ingroup geodesic
*/
static inline Eigen::Vector3d addVectorToGlobalPosition(
  const Eigen::Vector3d & global_position,
  const Eigen::Vector3f & vector_ned)
{
  const Eigen::Vector2d global_position_res = addVectorToGlobalPosition(
    static_cast<Eigen::Vector2d>(global_position.head(2)),
    static_cast<Eigen::Vector2f>(vector_ned.head(2)));
  const double d_alt = -vector_ned.z();
  return Eigen::Vector3d {global_position_res.x(), global_position_res.y(),
    global_position.z() + d_alt};
}

/** @}*/
}  // namespace px4_ros2
