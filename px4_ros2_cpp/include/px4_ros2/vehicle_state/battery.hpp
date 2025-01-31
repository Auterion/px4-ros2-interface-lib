/****************************************************************************
 * Copyright (c) 2023-2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/utils/subscription.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{
/** \ingroup vehicle_state
 *  @{
 */

/**
 * @brief Provides access to the vehicle's battery status
 *
 * @ingroup vehicle_state
 */
class Battery : public Subscription<px4_msgs::msg::BatteryStatus>
{
public:
  explicit Battery(Context & context)
  : Subscription<px4_msgs::msg::BatteryStatus>(context,
      "fmu/out/battery_status" + px4_ros2::getMessageNameVersion<px4_msgs::msg::BatteryStatus>()) {}

  /**
   * @brief Get the vehicle's battery voltage.
   *
   * @return the voltage [V], or 0 if unknown
   */
  float voltageV() const
  {
    return last().voltage_v;
  }

  /**
   * @brief Get the vehicle's battery current.
   *
   * @return the current [A], or -1 if unknown
   */
  float currentA() const
  {
    return last().current_a;
  }

  /**
   * @brief Get the vehicle's battery cumulative discharged capacity.
   *
   * @return the discharged capacity [mAh], or -1 if unknown
   */
  float dischargedCapacityMah() const
  {
    return last().discharged_mah;
  }

  /**
   * @brief Get the vehicle's battery remaining charge as a fraction.
   *
   * @return the remaining battery charge, 0.0 (empty) to 1.0 (full), or -1 if unknown.
   */
  float remaningFraction() const
  {
    return last().remaining;
  }

  /**
   * @brief Get the vehicle's battery cell count.
   *
   * @return the number of cells, 0 if unknown
   */
  uint8_t cellCount() const
  {
    return last().cell_count;
  }

  /**
   * @brief Get the voltages of each cell in the vehicle's battery.
   *
   * @return an Eigen vector containing the cell voltages. If the cell count is unknown, returns an empty vector.
   */
  Eigen::VectorXf cellVoltagesV() const
  {
    const px4_msgs::msg::BatteryStatus & battery = last();
    const uint8_t cell_count = battery.cell_count;

    if (cell_count == 0) {
      RCLCPP_WARN_THROTTLE(
        _node.get_logger(),
        *_node.get_clock(), 1000, "Failed to retrieve battery cell voltage: cell count unknown.");
      return Eigen::VectorXf();
    }

    Eigen::VectorXf cell_voltages(cell_count);
    for (uint8_t i = 0; i < cell_count; ++i) {
      cell_voltages(i) = battery.voltage_cell_v[i];
    }

    return cell_voltages;
  }

};

/** @}*/
} /* namespace px4_ros2 */
