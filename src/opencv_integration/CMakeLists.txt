cmake_minimum_required(VERSION 3.8)
project(opencv_integration)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Temukan package yang dibutuhkan
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Tambahkan executable node kita
add_executable(video_publisher_node src/video_publisher_node.cpp)

# Tautkan dependensi ke executable
ament_target_dependencies(video_publisher_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Aturan instalasi
install(TARGETS
  video_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Penutup
ament_package()