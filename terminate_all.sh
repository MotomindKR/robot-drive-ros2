#!/bin/bash
#
# turn_off_ros.sh ― run_all_ros.sh 로 띄운 모든 노드를 정리
#

# 먼저 Ctrl‑C(SIGNAL 2)와 동일한 SIGINT를 보내서 ‘정상 종료’ 시도
targets=(
  "laser_scan_box_filter"
  "scan_to_scan_filter_chain"
  "sllidar_node"
  "slam_toolbox"
  "robot_controller"
  "rplidar_node"
  "od2_controller_ros2"
  "ros2 launch od2_navigation od2_mapping.launch.py"
  "ros2 launch od2_navigation od2_localization.launch.py"
  "ros2 launch od2_node_manager od2_node_manager.launch.py"
  "od2_pose_publisher"
  "robot_data_publisher"
  "topic_list_server_node"
  "gripper_control"
  "rviz2"
  "topic_list_server"
  "od2_node_current_pose_sender"
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

echo "[✓] All ROS 2 nodes have been stopped."
