cmake_minimum_required(VERSION 3.8)
project(vision_ov_yolo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yolo_msgs REQUIRED) # Paket pesan kita tadi
find_package(OpenVINO REQUIRED)  # Cari OpenVINO

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Buat Executable
add_executable(yolo_detector 
  src/yolo_node.cpp
  src/yolo_engine.cpp
)

# Link Libraries
target_link_libraries(yolo_detector
  ${OpenCV_LIBS}
  openvino::runtime
)

# Link ROS Dependencies
ament_target_dependencies(yolo_detector
  rclcpp
  sensor_msgs
  cv_bridge
  yolo_msgs
)

# Install Executable
install(TARGETS yolo_detector
  DESTINATION lib/${PROJECT_NAME}
)

# Install Folder (Penting untuk akses file model/video)
install(DIRECTORY
  models
  data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()