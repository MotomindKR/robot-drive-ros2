cmake_minimum_required(VERSION 3.5)
project(od2_navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)

# Install launch files


ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  od2_navigation/map_name_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz config maps od2_navigation
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
