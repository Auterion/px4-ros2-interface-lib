cmake_minimum_required(VERSION 3.8)
project(example_mode_collision_avoidance_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(drone_path_planner REQUIRED)  # For custom messages

include_directories(include ${Eigen3_INCLUDE_DIRS})

# Executable
add_executable(example_mode_collision_avoidance
  src/main.cpp
)

ament_target_dependencies(example_mode_collision_avoidance
  rclcpp
  px4_ros2_cpp
  px4_msgs
  drone_path_planner
)

target_link_libraries(example_mode_collision_avoidance
  Eigen3::Eigen
)

# Install
install(TARGETS
  example_mode_collision_avoidance
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
