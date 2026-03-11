#!/usr/bin/env bash

# Check if mapname was provided
if [ -z "$1" ]; then
  echo "Usage: $0 <mapname_without_extension>"
  exit 1
fi

MAPNAME=$1

# Directory for logs
LOGDIR="./ros2_startup_logs"
mkdir -p "${LOGDIR}"

# Source your workspace
source ./ello_ws/install/setup.bash

# Launch each ROS 2 component in background, logging to files
ros2 launch od2_navigation od2_localization.launch.py map_file_name:=${MAPNAME} \
    > "${LOGDIR}/od2_localization.log" 2>&1 &

ros2 launch od2_node_manager od2_node_manager.launch.py mapname:=${MAPNAME}.json \
    > "${LOGDIR}/od2_node_manager.log" 2>&1 &

ros2 run od2_navigation map_name_publisher.py \
    --ros-args -p map_name:=${MAPNAME} \
    > "${LOGDIR}/map_name_publisher.log" 2>&1 &
    

wait

exit 0
