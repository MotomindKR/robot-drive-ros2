#!/usr/bin/env bash
#----------------------------------------------------------
# Save current SLAM map + pose‑graph + 빈 config JSON
#   Usage:  ./map_saver.sh <map_name>
#----------------------------------------------------------
set -euo pipefail

if [[ $# -lt 1 || -z "$1" ]]; then
  echo "Usage: $0 <map_name>"
  exit 1
fi
MAP_NAME="$1"

# ── 경로 --------------------------------------------------
MAP_DIR="./ello_ws/src/od2_navigation/maps"
MAP_DIR2="./ello_ws/install/od2_navigation/share/od2_navigation/maps"

mkdir -p "$MAP_DIR"
mkdir -p "$MAP_DIR2"

FULL_PATH="${MAP_DIR}/${MAP_NAME}"   # 확장자 없이!
FULL_PATH2="${MAP_DIR2}/${MAP_NAME}"   # 확장자 없이!

# ── SLAM Toolbox -----------------------------------------
# ① occupancy grid(.pgm/.yaml) 저장

#ros2 service call /slam_toolbox/save_map \
#  slam_toolbox/srv/SaveMap \
#  "{name: {data: '${FULL_PATH}'}}"

#ros2 service call /slam_toolbox/save_map \
#  slam_toolbox/srv/SaveMap \
#  "{name: {data: '${FULL_PATH2}'}}"

# ② pose‑graph(.data) 저장
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '${FULL_PATH}'}"

ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '${FULL_PATH2}'}"
