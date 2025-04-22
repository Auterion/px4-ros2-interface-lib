/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/utils/geodesic.hpp>
#include <px4_ros2/utils/message_version.hpp>

#include "map_projection_impl.hpp"

namespace px4_ros2
{

MapProjection::MapProjection(Context & context)
: _node(context.node())
{
  _map_projection_math = std::make_unique<MapProjectionImpl>();
  _vehicle_local_position_sub = _node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "fmu/out/vehicle_local_position" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>(), rclcpp::QoS(
      1).best_effort(),
    [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
      vehicleLocalPositionCallback(std::move(msg));
    });
}

MapProjection::~MapProjection() = default;

void MapProjection::vehicleLocalPositionCallback(px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
{
  const uint64_t timestamp_cur = msg->ref_timestamp;
  const uint64_t timestamp_ref = _map_projection_math->getProjectionReferenceTimestamp();
  if (!isInitialized()) {
    if (timestamp_cur != 0) {
      // Initialize map projection reference point
      _map_projection_math->initReference(
        msg->ref_lat, msg->ref_lon, msg->ref_alt,
        timestamp_cur
      );
    }
  } else if (timestamp_cur != timestamp_ref) {
    // Update reference point if it has changed
    _map_projection_math->initReference(
      msg->ref_lat, msg->ref_lon, msg->ref_alt,
      timestamp_cur
    );
    RCLCPP_WARN(_node.get_logger(), "Map projection reference point has been reset.");
  }
}

bool MapProjection::isInitialized() const
{
  return _map_projection_math->isInitialized();
}

void MapProjection::assertInitalized() const
{
  if (!isInitialized()) {
    throw std::runtime_error("Map projection impossible: uninitialized.");
  }
}

Eigen::Vector2f MapProjection::globalToLocal(const Eigen::Vector2d & global_position) const
{
  assertInitalized();
  return _map_projection_math->globalToLocal(global_position);
}

Eigen::Vector3f MapProjection::globalToLocal(const Eigen::Vector3d & global_position) const
{
  assertInitalized();
  return _map_projection_math->globalToLocal(global_position);
}

Eigen::Vector2d MapProjection::localToGlobal(const Eigen::Vector2f & local_position) const
{
  assertInitalized();
  return _map_projection_math->localToGlobal(local_position);
}

Eigen::Vector3d MapProjection::localToGlobal(const Eigen::Vector3f & local_position) const
{
  assertInitalized();
  return _map_projection_math->localToGlobal(local_position);
}

float horizontalDistanceToGlobalPosition(
  const Eigen::Vector2d & global_position_now,
  const Eigen::Vector2d & global_position_next)
{
  const double lat_now_rad = degToRad(global_position_now.x());
  const double lat_next_rad = degToRad(global_position_next.x());

  const double d_lat = lat_next_rad - lat_now_rad;
  const double d_lon = degToRad(global_position_next.y()) - degToRad(global_position_now.y());

  const double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + sin(d_lon / 2.0) * sin(d_lon / 2.0) * cos(
    lat_now_rad) * cos(lat_next_rad);

  const double c = atan2(sqrt(a), sqrt(1.0 - a));

  return static_cast<float>(kRadiusOfEarth * 2.0 * c);
}

float distanceToGlobalPosition(
  const Eigen::Vector3d & global_position_now,
  const Eigen::Vector3d & global_position_next)
{
  const float dxy = horizontalDistanceToGlobalPosition(global_position_now, global_position_next);
  const float dz = static_cast<float>(global_position_now.z() - global_position_next.z());

  return sqrtf(dxy * dxy + dz * dz);
}

float headingToGlobalPosition(
  const Eigen::Vector2d & global_position_now,
  const Eigen::Vector2d & global_position_next)
{
  const double lat_now_rad = degToRad(global_position_now.x());
  const double lat_next_rad = degToRad(global_position_next.x());

  const double cos_lat_next = cos(lat_next_rad);
  const double d_lon = degToRad(global_position_next.y() - global_position_now.y());

  const float y = static_cast<float>(sin(d_lon) * cos_lat_next);
  const float x =
    static_cast<float>(cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos_lat_next *
    cos(d_lon));

  return wrapPi(atan2f(y, x));
}

Eigen::Vector2f vectorToGlobalPosition(
  const Eigen::Vector2d & global_position_now,
  const Eigen::Vector2d & global_position_next)
{
  Eigen::Vector2f vector_ne;

  const double lat_now_rad = degToRad(global_position_now.x());
  const double lat_next_rad = degToRad(global_position_next.x());
  const double d_lon = degToRad(global_position_next.y()) - degToRad(global_position_now.y());

  vector_ne.x() =
    static_cast<float>(kRadiusOfEarth *
    (cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos(
      lat_next_rad) * cos(d_lon)));
  vector_ne.y() = static_cast<float>(kRadiusOfEarth * sin(d_lon) * cos(lat_next_rad));

  return vector_ne;
}

Eigen::Vector2d globalPositionFromLineAndDist(
  const Eigen::Vector2d & global_position_line_start,
  const Eigen::Vector2d & global_position_line_end,
  float dist_from_start)
{
  float heading = headingToGlobalPosition(global_position_line_start, global_position_line_end);
  return globalPositionFromHeadingAndDist(global_position_line_start, heading, dist_from_start);
}

Eigen::Vector2d globalPositionFromHeadingAndDist(
  const Eigen::Vector2d & global_position_now,
  float heading, float dist)
{
  Eigen::Vector2d global_position_target;

  heading = wrapPi(heading);

  const double radius_ratio = static_cast<double>(dist) / kRadiusOfEarth;

  const double lat_now_rad = degToRad(global_position_now.x());
  const double lon_now_rad = degToRad(global_position_now.y());

  const double lat_target_rad = asin(
    sin(lat_now_rad) * cos(radius_ratio) + cos(lat_now_rad) * sin(radius_ratio) * cos(
      static_cast<double>(heading)));
  const double lon_target_rad = lon_now_rad + atan2(
    sin(static_cast<double>(heading)) * sin(radius_ratio) * cos(lat_now_rad),
    cos(radius_ratio) - sin(lat_now_rad) * sin(lat_target_rad));

  global_position_target.x() = radToDeg(lat_target_rad);
  global_position_target.y() = radToDeg(lon_target_rad);

  return global_position_target;
}

Eigen::Vector2d addVectorToGlobalPosition(
  const Eigen::Vector2d & global_position,
  const Eigen::Vector2f & vector_ne)
{
  Eigen::Vector2d global_position_res;

  double lat_now_rad = degToRad(global_position.x());
  double lon_now_rad = degToRad(global_position.y());

  global_position_res.x() =
    radToDeg(lat_now_rad + static_cast<double>(vector_ne.x()) / kRadiusOfEarth);
  global_position_res.y() =
    radToDeg(
    lon_now_rad + static_cast<double>(vector_ne.y()) /
    (kRadiusOfEarth * cos(lat_now_rad)));

  return global_position_res;
}

}  // namespace px4_ros2
