cmake_minimum_required(VERSION 3.5)
project(example_global_navigation_cpp)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

include_directories(include ${Eigen3_INCLUDE_DIRS})
add_executable(example_global_navigation
        src/main.cpp)
target_link_libraries(example_global_navigation PUBLIC Eigen3::Eigen px4_ros2_cpp::px4_ros2_cpp rclcpp::rclcpp)

install(TARGETS
        example_global_navigation
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
