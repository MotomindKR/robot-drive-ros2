set(PROJECT_NAME diff_rob_bringup)
cmake_minimum_required(VERSION 3.10)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Include directories
include_directories(src)

# Find packages
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Add executable
add_executable(${PROJECT_NAME} main.cc src/robotcontroller.cc)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2_ros
  geometry_msgs
  nav_msgs
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
