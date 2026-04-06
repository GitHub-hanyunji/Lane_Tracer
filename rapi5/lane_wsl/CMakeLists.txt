cmake_minimum_required(VERSION 3.5)
project(lane)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

#build
add_executable(psub
  src/main.cpp
  src/lane.cpp
  )

ament_target_dependencies(psub rclcpp sensor_msgs cv_bridge OpenCV std_msgs geometry_msgs)

# Install
install(TARGETS
  psub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
