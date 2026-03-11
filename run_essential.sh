#!/usr/bin/env bash

# Directory for logs
LOGDIR="./ros2_startup_logs"
mkdir -p "${LOGDIR}"

source ./ello_ws/install/setup.bash


# Launch each ROS2 component in background, logging to files
ros2 launch rplidar_ros rplidar_c1_launch.py \
    > "${LOGDIR}/rplidar_c1.log" 2>&1 &

ros2 run diff_rob_bringup diff_rob_bringup \
    > "${LOGDIR}/od2_controller.log" 2>&1 &

ros2 run robot_data_publisher robot_data_publisher \
    > "${LOGDIR}/data_pub.log" 2>&1 &

ros2 run topic_list_server topic_list_server_node \
    > "${LOGDIR}/topic_list.log" 2>&1 &


# Wait for background jobs to finish
wait

exit 0
