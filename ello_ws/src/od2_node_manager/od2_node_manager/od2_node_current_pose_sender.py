#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import json
import math
from pathlib import Path
from rclpy.parameter import Parameter

class CurrentPoseSender(Node):
    def __init__(self):
        super().__init__('current_pose_sender')

        # publisher
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # ── Read mapname parameter ─────────────────────────────
        mapname = self.declare_parameter('mapname', 'home.json').get_parameter_value().string_value

        # ── 홈 노드 읽기 ───────────────────────────────────────
        base_config_path = '/home/gyeum/robot_arm_ws/src/od2_node_manager/config'
        cfg_path = Path(base_config_path) / mapname
        self.get_logger().info(f"Loading initial pose from: {cfg_path}")

        with cfg_path.open() as f:
            cfg = json.load(f)

        self.home = next(
            (n for n in cfg.get('nodes', []) if n.get('type') == 'home'),
            None
        )

        if not self.home:
            self.get_logger().warn('type "home" 노드를 찾을 수 없습니다. 종료하지 않고 대기만 합니다.')
            return

        # publish 전송 여부 플래그
        self.sent = False

        # 100 ms 주기로 구독자‑체크 & publish 시도
        self.timer = self.create_timer(0.1, self.try_publish_initial_pose)

    # ── 주기적으로 호출되어 구독자 유무 확인 후 publish ───────────
    def try_publish_initial_pose(self):
        # 이미 전송했다면 타이머 중지
        if self.sent:
            self.timer.cancel()
            return

        sub_cnt = self.pose_pub.get_subscription_count()
        if sub_cnt == 0:
            # 아직 아무 노드도 /initialpose 를 구독하지 않음
            self.get_logger().debug('구독자를 기다리는 중 ...')
            return

        # 최소 1개 이상 구독자가 있으니 publish
        self.publish_initial_pose()
        self.sent = True

        # 0.3 초 뒤에 shutdown → 메시지가 DDS 버퍼를 통해 완전히 송신될 시간 확보
        self.create_timer(0.3, lambda: rclpy.shutdown())

    # ── 실제 메시지 빌드 & 전송 ─────────────────────────────────
    def publish_initial_pose(self):
        n = self.home
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.pose.position.x = n['x']
        pose.pose.pose.position.y = n['y']
        pose.pose.pose.position.z = n.get('z', 0.0)

        yaw = n['theta']
        pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw / 2.0)

        pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.0, 0.0, 0.0, 0.068
        ]

        self.pose_pub.publish(pose)
        self.get_logger().info(f'home 노드(id={n["id"]}) 초기 pose 전송 완료.')


def main():
    rclpy.init()
    node = CurrentPoseSender()
    rclpy.spin(node)
    # shutdown()은 내부 타이머에서 호출됨
    node.destroy_node()


if __name__ == '__main__':
    main()
