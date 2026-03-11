# Motomind Ello Public ROS2 Package
## Introduction
Ello is the platform of Motomind's autonomous driving robot. Via ROS2, we provide navigation system which includes node/edge manager. Also the platform includes refined data logging that can be used in a variety of applications.

## Release Notes
(WIP)

## Supported Platforms
1. Linux 22.04 / ROS2 Humble
2. Linux 20.04 / ROS2 Foxy

## Contained ROS2 Packages
1. diff_rob_bringup
2. od2_navigation
3. od2_node_manager
4. robot_data_publisher
5. topic_list_server

## Quick Start
1. Install proper ROS2 matches with your Linux version.
ROS2 Foxy: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
ROS2 Humble: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

In case of ROS2 Humble, you might use the contained script.

    ./script/install_ros2_humble.sh

2. (optional) Set ROS2 alias in ~/.bashrc for convenience.

After setting alias, you can source ROS2 env in terminal by command {source_ros2}

    ./script/set_ros_alias.sh

3. Build ROS2 package
   
This platform contains 5 packages.

    cd robot-drive-ros/ello_ws
    colcon build --packages-select {package_name}
    e.g. colcon build --packages-select od2_navigation 

4. Execute script

Execute contained sample scripts to execute program.

    ./run_autodrive.sh
    ./run_mapping.sh

## Contact

If you need any support, please contact us anytime.

    contact@motomind.co.kr
   
