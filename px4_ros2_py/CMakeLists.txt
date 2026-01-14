cmake_minimum_required(VERSION 3.16)
project(px4_ros2_py)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

ament_python_install_package(px4_ros2)

pybind11_add_module(
        px4_ros2_py
        src/px4_ros2/bind_main.cpp
        src/px4_ros2/components/mode.cpp
        src/px4_ros2/components/mode_executor.cpp
        src/px4_ros2/components/node_wrapper.cpp
        src/px4_ros2/control/setpoint_types/multicopter/goto.cpp
        src/px4_ros2/odometry/local_position.cpp
        src/px4_ros2/utils/geometry.cpp
)

target_link_libraries(
        px4_ros2_py
        PRIVATE
        px4_ros2_cpp::px4_ros2_cpp
        rclcpp::rclcpp
)
target_include_directories(px4_ros2_py PRIVATE include)

install(TARGETS px4_ros2_py
        DESTINATION "${PYTHON_INSTALL_DIR}/px4_ros2"
)

ament_package()