cmake_minimum_required(VERSION 3.5)
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

include_directories(include SYSTEM ${Eigen3_INCLUDE_DIRS})
set(HEADER_FILES
        include/px4_ros2/common/setpoint_base.hpp
        include/px4_ros2/components/events.hpp
        include/px4_ros2/components/health_and_arming_checks.hpp
        include/px4_ros2/components/manual_control_input.hpp
        include/px4_ros2/components/message_compatibility_check.hpp
        include/px4_ros2/components/mode.hpp
        include/px4_ros2/components/mode_executor.hpp
        include/px4_ros2/components/overrides.hpp
        include/px4_ros2/components/wait_for_fmu.hpp
        include/px4_ros2/control/peripheral_actuators.hpp
        include/px4_ros2/control/setpoint_types/direct_actuators.hpp
        include/px4_ros2/control/setpoint_types/goto.hpp
        include/px4_ros2/control/setpoint_types/experimental/attitude.hpp
        include/px4_ros2/control/setpoint_types/experimental/rates.hpp
        include/px4_ros2/control/setpoint_types/experimental/rover_cruise_control.hpp
        include/px4_ros2/control/setpoint_types/experimental/trajectory.hpp
        include/px4_ros2/navigation/experimental/global_position_measurement_interface.hpp
        include/px4_ros2/navigation/experimental/local_position_measurement_interface.hpp
        include/px4_ros2/navigation/experimental/navigation_interface_base.hpp
        include/px4_ros2/odometry/global_position.hpp
        include/px4_ros2/odometry/local_position.hpp
        include/px4_ros2/utils/frame_conversion.hpp
        include/px4_ros2/utils/geometry.hpp
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
        src/control/setpoint_types/goto.cpp
        src/control/setpoint_types/experimental/attitude.cpp
        src/control/setpoint_types/experimental/rates.cpp
        src/control/setpoint_types/experimental/rover_cruise_control.cpp
        src/control/setpoint_types/experimental/trajectory.cpp
        src/navigation/experimental/global_position_measurement_interface.cpp
        src/navigation/experimental/local_position_measurement_interface.cpp
        src/odometry/global_position.cpp
        src/odometry/local_position.cpp
)
ament_target_dependencies(px4_ros2_cpp ament_index_cpp Eigen3 rclcpp px4_msgs)

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
    ament_target_dependencies(integration_utils rclcpp px4_msgs)

    ament_add_gtest(integration_tests
            test/integration/arming_check.cpp
            test/integration/global_navigation.cpp
            test/integration/local_navigation.cpp
            test/integration/mode.cpp
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
    ament_target_dependencies(unit_utils Eigen3)

    ament_add_gtest(${PROJECT_NAME}_unit_tests
            test/unit/global_navigation.cpp
            test/unit/local_navigation.cpp
            test/unit/main.cpp
            test/unit/modes.cpp
            test/unit/utils/frame_conversion.cpp
            test/unit/utils/geometry.cpp
    )
    target_include_directories(${PROJECT_NAME}_unit_tests PRIVATE ${CMAKE_CURRENT_LIST_DIR})
    target_link_libraries(${PROJECT_NAME}_unit_tests ${PROJECT_NAME} unit_utils)
    ament_target_dependencies(${PROJECT_NAME}_unit_tests
            rclcpp px4_msgs
    )
endif()

ament_package()
