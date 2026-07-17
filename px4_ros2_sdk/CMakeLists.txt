cmake_minimum_required(VERSION 3.16)
project(px4_ros2_sdk)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_ros2_sdk_interfaces REQUIRED)

add_library(px4_ros2_sdk
        src/result.cpp
        src/flight_server.cpp
        src/simple_flight.cpp
)
# The sources carry no __declspec export annotations, so on MSVC the DLL would
# otherwise export nothing and no import library is produced, which breaks
# linking the test executable. No effect on other platforms.
set_target_properties(px4_ros2_sdk PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
target_include_directories(px4_ros2_sdk PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
# Link the namespaced targets so their headers are treated as SYSTEM includes
# and do not trip the -Werror above.
target_link_libraries(px4_ros2_sdk PUBLIC
        px4_ros2_cpp::px4_ros2_cpp
        rclcpp::rclcpp
        rclcpp_action::rclcpp_action
        rclcpp_lifecycle::rclcpp_lifecycle
        rclcpp_components::component
        ${lifecycle_msgs_TARGETS}
        ${px4_msgs_TARGETS}
        ${px4_ros2_sdk_interfaces_TARGETS}
)

rclcpp_components_register_node(px4_ros2_sdk
        PLUGIN "px4_ros2::sdk::FlightServer"
        EXECUTABLE flight_server_node
)

ament_export_targets(px4_ros2_sdk HAS_LIBRARY_TARGET)
ament_export_dependencies(
        px4_ros2_cpp
        rclcpp
        rclcpp_action
        rclcpp_lifecycle
        lifecycle_msgs
        px4_msgs
        px4_ros2_sdk_interfaces
)

install(
        DIRECTORY include/px4_ros2_sdk
        DESTINATION include
)
install(
        TARGETS px4_ros2_sdk
        EXPORT px4_ros2_sdk
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_lint_auto_find_test_dependencies()

    # Named *_unit_tests so the CI `ctest -R unit_tests` filter runs it.
    ament_add_gtest(px4_ros2_sdk_unit_tests test/test_sdk_smoke.cpp)
    if(TARGET px4_ros2_sdk_unit_tests)
        target_link_libraries(px4_ros2_sdk_unit_tests
                px4_ros2_sdk
                rclcpp::rclcpp
                rclcpp_action::rclcpp_action
                rclcpp_lifecycle::rclcpp_lifecycle
                ${lifecycle_msgs_TARGETS}
                ${px4_ros2_sdk_interfaces_TARGETS}
        )
    endif()
endif()

ament_package()
