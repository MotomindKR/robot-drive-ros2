#!/bin/bash
#
# turn_off_ros.sh ― run_all_ros.sh 로 띄운 모든 노드를 정리
#

# 먼저 Ctrl‑C(SIGNAL 2)와 동일한 SIGINT를 보내서 ‘정상 종료’ 시도
targets=(
  "ros2 launch od2_navigation od2_localization.launch.py"
  "ros2 launch od2_node_manager od2_node_manager.launch.py"
  "slam_toolbox"
  "od2_pose_publisher"
  "od2_node_manager_node"
  "od2_edge_follower_node"
  "od2_virtual_bumper_node"
  "current_pose_sender"
  "map_name_publisher"
  "amcl"
)

echo "[*] Sending SIGINT to ROS 2 processes …"
for t in "${targets[@]}"; do
  pkill -SIGINT -f "$t"
done

# 잠깐 기다린 뒤 아직 남아 있으면 강제 종료
sleep 3

echo "[*] Forcing stubborn processes to exit …"
for t in "${targets[@]}"; do
  pkill -9 -f "$t" 2>/dev/null
done

# 자동으로 열었던 gnome‑terminal 탭도 정리(선택)
pkill -SIGTERM -f "gnome-terminal.*ros2" 2>/dev/null

echo "[✓] ROS 2 mapping nodes have been stopped."
