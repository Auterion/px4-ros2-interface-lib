/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_ros2/navigation/experimental/navigation_interface_base.hpp>

namespace px4_ros2
{
/** \ingroup navigation_experimental
 *  @{
 */

/**
 * @struct GlobalPositionMeasurement
 * @brief Represents a global position measurement to be passed to `GlobalPositionMeasurementInterface::update`.
 *
 * This struct holds information about the global position measurement, including: the timestamp of the sample, latitude and longitude, altitude in the MSL frame, and their associated variances.
 * @see GlobalPositionMeasurementInterface::update
 */
struct GlobalPositionMeasurement
{
  /** @brief Timestamp of the sample. */
  rclcpp::Time timestamp_sample {};

  /** @brief Latitude and longitude. */
  std::optional<Eigen::Vector2d> lat_lon {std::nullopt};
  /** @brief Variance of horizontal position error (meters). */
  std::optional<float> horizontal_variance {std::nullopt};

  /** @brief Altitude in the MSL frame. */
  std::optional<float> altitude_msl {std::nullopt};
  /** @brief Variance of vertical position error (meters). */
  std::optional<float> vertical_variance {std::nullopt};
};

/**
 * @brief Base class for a global position measurement provider
 * @ingroup navigation_experimental
 */
class GlobalPositionMeasurementInterface : public PositionMeasurementInterfaceBase
{
public:
  explicit GlobalPositionMeasurementInterface(rclcpp::Node & node);
  ~GlobalPositionMeasurementInterface() override = default;

  /**
   * @brief Publishes a global position measurement to the FMU.
   * Throws an exception if the following conditions are not met:
   * 1. The sample timestamp is defined.
   * 2. Values do not have a NAN.
   * 3. If a measurement value is provided, its associated variance value is well defined.
   * @param global_position_measurement The global position measurement to publish.
   * @throws px4_ros2::NavigationInterfaceInvalidArgument if the conditions above are not met.
   */
  void update(const GlobalPositionMeasurement & global_position_measurement) const;

/**
 * @brief Notify the FMU that the global position estimate has been reset.
 *
 * Increments the reset counter for horizontal position (latitude and longitude) to signal the EKF
 * of a discontinuity in external position data (e.g., loss of tracking or reinitialization). Future measurement
 * updates will contain the incremented counter. This prevents the EKF from rejecting future measurement updates
 * after an inconsistency in data.
 */
  inline void reset() {++_lat_lon_reset_counter;}

private:
  /**
   * @brief Check that at least one measurement value is defined.
   */
  bool isMeasurementNonEmpty(const GlobalPositionMeasurement & measurement) const;

  /**
   * @brief Check that if a measurement value is defined, its variance is also defined and strictly greater than zero.
   */
  bool isVarianceValid(const GlobalPositionMeasurement & measurement) const;

  /**
   * @brief Check that if a measurement value is defined, its associated frame is not *FRAME_UNKNOWN.
   */
  bool isFrameValid(const GlobalPositionMeasurement & measurement) const;

  /**
   * @brief Check that if a measurement value is defined, none of its fields are NAN.
   */
  bool isValueNotNAN(const GlobalPositionMeasurement & measurement) const;

  rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr _aux_global_position_pub;

  uint8_t _lat_lon_reset_counter{0};  /** Counter for reset events on horizontal position coordinates */
  // uint8_t _altitude_frame;
};

/** @}*/
} // namespace px4_ros2
