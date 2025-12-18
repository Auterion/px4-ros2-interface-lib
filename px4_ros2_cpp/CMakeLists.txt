cmake_minimum_required(VERSION 3.16)
project(px4_ros2_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
set(BUILD_SHARED_LIBS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

if($ENV{ROS_DISTRO} STREQUAL "humble" OR $ENV{ROS_DISTRO} STREQUAL "galactic")
    add_compile_definitions(ROS2_HUMBLE_OR_GALACTIC)
endif()

# Galactic lacks add_pre_shutdown_callback; assume present otherwise.
set(HAS_RCLCPP_PRE_SHUTDOWN ON)
if($ENV{ROS_DISTRO} STREQUAL "galactic")
    set(HAS_RCLCPP_PRE_SHUTDOWN OFF)
endif()

ament_index_get_resource(MESSAGE_FILES "rosidl_interfaces" px4_msgs)

# Split the message files into a list
string(REPLACE "\n" ";" msg_file_list "${MESSAGE_FILES}")

set(PX4_MSG_FILES "")
foreach(file ${msg_file_list})
  if("${file}" MATCHES "^msg/(.*\\.msg)$")
    string(REGEX REPLACE "^msg/" "" cleaned_file "${file}")
    list(APPEND PX4_MSG_FILES "${cleaned_file}")
  endif()
endforeach()

# Convert list to Python-style list string
string(REPLACE ";" "','" MESSAGE_NAMES_QUOTED "${PX4_MSG_FILES}")
set(MESSAGE_NAMES_PYTHON "[\'${MESSAGE_NAMES_QUOTED}\']")

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/px4_all_messages.hpp
  COMMAND ${Python3_EXECUTABLE} -m em
    -D message_names=\"${MESSAGE_NAMES_PYTHON}\"
    ${CMAKE_CURRENT_SOURCE_DIR}/src/components/px4_all_messages.hpp.em
    > ${CMAKE_CURRENT_BINARY_DIR}/px4_all_messages.hpp
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/components/px4_all_messages.hpp.em
  COMMENT "Generating px4_all_messages.hpp using empy"
)

add_custom_target(generate_px4_header DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/px4_all_messages.hpp)

include_directories(include SYSTEM ${Eigen3_INCLUDE_DIRS})
set(HEADER_FILES
        include/px4_ros2/common/setpoint_base.hpp
        include/px4_ros2/components/events.hpp
        include/px4_ros2/components/health_and_arming_checks.hpp
        include/px4_ros2/components/manual_control_input.hpp
        include/px4_ros2/components/message_compatibility_check.hpp
        include/px4_ros2/components/mode.hpp
        include/px4_ros2/components/mode_executor.hpp
        include/px4_ros2/components/node_with_mode.hpp
        include/px4_ros2/components/overrides.hpp
        include/px4_ros2/components/shared_subscription.hpp
        include/px4_ros2/components/wait_for_fmu.hpp
        include/px4_ros2/control/peripheral_actuators.hpp
        include/px4_ros2/control/setpoint_types/direct_actuators.hpp
        include/px4_ros2/control/setpoint_types/multicopter/goto.hpp
        include/px4_ros2/control/setpoint_types/fixedwing/lateral_longitudinal.hpp
        include/px4_ros2/control/setpoint_types/experimental/attitude.hpp
        include/px4_ros2/control/setpoint_types/experimental/rates.hpp
        include/px4_ros2/control/setpoint_types/experimental/trajectory.hpp
        include/px4_ros2/control/setpoint_types/experimental/rover/position.hpp
        include/px4_ros2/control/setpoint_types/experimental/rover/speed_attitude.hpp
        include/px4_ros2/control/setpoint_types/experimental/rover/speed_rate.hpp
        include/px4_ros2/control/setpoint_types/experimental/rover/speed_steering.hpp
        include/px4_ros2/control/setpoint_types/experimental/rover/throttle_attitude.hpp
        include/px4_ros2/control/setpoint_types/experimental/rover/throttle_rate.hpp
        include/px4_ros2/control/setpoint_types/experimental/rover/throttle_steering.hpp
        include/px4_ros2/control/vtol.hpp
        include/px4_ros2/mission/actions/action.hpp
        include/px4_ros2/mission/actions/default_actions.hpp
        include/px4_ros2/mission/mission.hpp
        include/px4_ros2/mission/mission_executor.hpp
        include/px4_ros2/mission/trajectory/multicopter/waypoint_trajectory_executor.hpp
        include/px4_ros2/mission/trajectory/trajectory_executor.hpp
        include/px4_ros2/navigation/experimental/global_position_measurement_interface.hpp
        include/px4_ros2/navigation/experimental/local_position_measurement_interface.hpp
        include/px4_ros2/navigation/experimental/navigation_interface_base.hpp
        include/px4_ros2/odometry/attitude.hpp
        include/px4_ros2/odometry/global_position.hpp
        include/px4_ros2/odometry/local_position.hpp
        include/px4_ros2/odometry/angular_velocity.hpp
        include/px4_ros2/odometry/airspeed.hpp
        include/px4_ros2/utils/frame_conversion.hpp
        include/px4_ros2/utils/geodesic.hpp
        include/px4_ros2/utils/geometry.hpp
        include/px4_ros2/utils/message_version.hpp
        include/px4_ros2/utils/visit.hpp
        include/px4_ros2/vehicle_state/battery.hpp
        include/px4_ros2/vehicle_state/home_position.hpp
        include/px4_ros2/vehicle_state/land_detected.hpp
        include/px4_ros2/vehicle_state/vehicle_status.hpp
        include/px4_ros2/vehicle_state/vtol_status.hpp
        include/px4_ros2/third_party/nlohmann/json.hpp
        include/px4_ros2/third_party/nlohmann/json_fwd.hpp
)

add_library(px4_ros2_cpp
        ${HEADER_FILES}
        src/components/health_and_arming_checks.cpp
        src/components/manual_control_input.cpp
        src/components/message_compatibility_check.cpp
        src/components/mode.cpp
        src/components/mode_executor.cpp
        src/components/overrides.cpp
        src/components/registration.cpp
        src/components/wait_for_fmu.cpp
        src/control/peripheral_actuators.cpp
        src/control/setpoint_types/direct_actuators.cpp
        src/control/setpoint_types/experimental/attitude.cpp
        src/control/setpoint_types/experimental/rates.cpp
        src/control/setpoint_types/experimental/rover/position.cpp
        src/control/setpoint_types/experimental/rover/speed_attitude.cpp
        src/control/setpoint_types/experimental/rover/speed_rate.cpp
        src/control/setpoint_types/experimental/rover/speed_steering.cpp
        src/control/setpoint_types/experimental/rover/throttle_attitude.cpp
        src/control/setpoint_types/experimental/rover/throttle_rate.cpp
        src/control/setpoint_types/experimental/rover/throttle_steering.cpp
        src/control/setpoint_types/experimental/trajectory.cpp
        src/control/setpoint_types/fixedwing/lateral_longitudinal.cpp
        src/control/setpoint_types/multicopter/goto.cpp
        src/control/vtol.cpp
        src/mission/actions/default_actions.cpp
        src/mission/mission.cpp
        src/mission/mission_executor.cpp
        src/mission/trajectory/multicopter/waypoint_trajectory_executor.cpp
        src/navigation/experimental/global_position_measurement_interface.cpp
        src/navigation/experimental/local_position_measurement_interface.cpp
        src/odometry/airspeed.cpp
        src/odometry/angular_velocity.cpp
        src/odometry/attitude.cpp
        src/odometry/global_position.cpp
        src/odometry/local_position.cpp
        src/utils/geodesic.cpp
        src/utils/map_projection_impl.cpp
)
add_dependencies(px4_ros2_cpp generate_px4_header)
target_include_directories(px4_ros2_cpp PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
target_link_libraries(px4_ros2_cpp PUBLIC ament_index_cpp::ament_index_cpp Eigen3::Eigen rclcpp::rclcpp ${px4_msgs_TARGETS})
target_compile_definitions(px4_ros2_cpp PUBLIC HAS_RCLCPP_PRE_SHUTDOWN=$<BOOL:${HAS_RCLCPP_PRE_SHUTDOWN}>)

ament_export_targets(px4_ros2_cpp HAS_LIBRARY_TARGET)
ament_export_dependencies(
        px4_msgs
        rclcpp
        eigen3_cmake_module
        Eigen3
)
install(
        DIRECTORY include/px4_ros2
        DESTINATION include
)
install(
        TARGETS px4_ros2_cpp
        EXPORT px4_ros2_cpp
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_lint_auto_find_test_dependencies()

    # Integration tests
    add_library(integration_utils
            test/integration/util.cpp
    )
    include_directories(include)
    target_link_libraries(integration_utils PUBLIC rclcpp::rclcpp ${px4_msgs_TARGETS})

    ament_add_gtest(integration_tests
            test/integration/arming_check.cpp
            test/integration/global_navigation.cpp
            test/integration/local_navigation.cpp
            test/integration/mode.cpp
            test/integration/mission.cpp
            test/integration/mode_executor.cpp
            test/integration/overrides.cpp
    )
    if(TARGET integration_tests)
        target_link_libraries(integration_tests ${PROJECT_NAME} integration_utils)
        target_include_directories(integration_tests PRIVATE include)
    endif()


    # Unit tests
    add_library(unit_utils
            test/unit/utils/util.cpp
    )
    include_directories(include)
    target_link_libraries(unit_utils PUBLIC Eigen3::Eigen)

    ament_add_gtest(${PROJECT_NAME}_unit_tests
            test/unit/global_navigation.cpp
            test/unit/local_navigation.cpp
            test/unit/main.cpp
            test/unit/mission_execution.cpp
            test/unit/modes.cpp
            test/unit/shared_subscription.cpp
            test/unit/utils/frame_conversion.cpp
            test/unit/utils/geodesic.cpp
            test/unit/utils/geometry.cpp
            test/unit/utils/map_projection_impl.cpp
    )
    target_include_directories(${PROJECT_NAME}_unit_tests PRIVATE ${CMAKE_CURRENT_LIST_DIR})
    target_link_libraries(${PROJECT_NAME}_unit_tests ${PROJECT_NAME} unit_utils rclcpp::rclcpp ${px4_msgs_TARGETS})
endif()

ament_package()
