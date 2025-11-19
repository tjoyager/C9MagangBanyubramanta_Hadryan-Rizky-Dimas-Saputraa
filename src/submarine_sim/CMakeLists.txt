cmake_minimum_required(VERSION 3.8)
project(submarine_sim)

# Default ke C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_dev REQUIRED)

# Build Plugin Controller
add_library(rov_controller SHARED src/rov_controller.cpp)
ament_target_dependencies(rov_controller
  rclcpp
  gazebo_ros
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  gazebo_dev
)

install(TARGETS rov_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install folder assets
install(DIRECTORY
  launch description worlds meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()