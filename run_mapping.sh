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
ros2 launch od2_navigation od2_mapping.launch.py map_file_name:=${MAPNAME} \
    > "${LOGDIR}/od2_mapping.log" 2>&1 &

ros2 run od2_navigation map_name_publisher.py \
    --ros-args -p map_name:=${MAPNAME} \
    > "${LOGDIR}/map_name_publisher.log" 2>&1 &

ros2 run od2_node_manager od2_pose_publisher \
    > "${LOGDIR}/od2_pose_publisher.log" 2>&1 &

# Wait for all background jobs to finish (optional)

CONFIG_DIR="./ello_ws/src/od2_node_manager/config"
mkdir -p "$CONFIG_DIR"

  CONFIG_FILE="${CONFIG_DIR}/${MAPNAME}.json"
  cat > "$CONFIG_FILE" <<EOF
{
  "nodes": [],
  "edges": []
}
EOF
  echo "Created config file: $CONFIG_FILE"

wait

exit 0
