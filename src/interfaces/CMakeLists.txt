cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detect.msg"
  "msg/ControllerCommand.msg"  # <--- TAMBAHKAN BARIS INI
)

ament_package()