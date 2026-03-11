import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped
import math
import json
import threading
import socket  # TCP 메시지 전송용

# 외부 TCP 서버 설정 (IP와 포트는 필요에 따라 수정)
EXTERNAL_TCP_SERVER_IP = '192.168.0.99'  # 예시 IP
EXTERNAL_TCP_SERVER_PORT = 19303           # 예시 포트

EXTERNAL_PATH_PORT = 19304  # 경로 정보를 받을 포트

class EdgeVelDriver(Node):
    def __init__(self):
        super().__init__('edge_vel_driver')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_sub = self.create_subscription(
            TransformStamped,
            'current_pose',
            self.pose_callback,
            10
        )
        self.state_sub = self.create_subscription(
            String,
            'robot_state',
            self.state_callback,
            10
        )

        # 1) Load path planning config (the topological map)
        self.config_path = '/home/gyeum/robot_arm_ws/src/od2_node_manager/config/config.json'
        self.load_map_config()

        # 2) Load driving parameters (gains, speeds, etc.)
        self.driver_config_path = '/home/gyeum/robot_arm_ws/src/od2_node_manager/config/edge_driver_config.json'
        self.driver_params = {}
        self.load_driver_config()

        # Internal state
        self.robot_pose = None
        self.path = []            # 경로 노드 ID 리스트
        self.current_index = 0
        self.state = 'idle'
        self.vx = 0.0
        self.vyaw = 0.0
        self.prev_yaw_err = 0.0
        self.prev_lateral_error = 0.0
        self.prev_heading_err = 0.0
        self.external_state = 'free'
        self.current_speed_factor = 1.0
        self.desired_speed_factor = 1.0
        self.recovery_step = 0
        self.final_yaw = 0.0

        self.timer = self.create_timer(0.05, self.control_loop)

        # 기존 사용자 입력 대신 TCP로 경로 정보를 받기 위한 스레드
        threading.Thread(target=self.tcp_input_thread, daemon=True).start()

    # ---------------------------
    #   LOAD CONFIG FILES
    # ---------------------------
    def load_map_config(self):
        with open(self.config_path) as f:
            self.config = json.load(f)

    def load_driver_config(self):
        with open(self.driver_config_path) as f:
            self.driver_params = json.load(f)

    # ---------------------------
    #   TCP 메시지 전송 함수
    # ---------------------------
    def send_tcp_message(self, message):
        """
        외부 TCP 서버로 메시지를 전송합니다.
        한 번의 연결로 메시지 전송 후 연결을 종료합니다.
        """
        try:
            sock = socket.create_connection(
                (EXTERNAL_TCP_SERVER_IP, EXTERNAL_TCP_SERVER_PORT), 
                timeout=5
            )
            sock.sendall(message.encode('utf-8'))
            sock.close()
            self.get_logger().info(f"Sent TCP message: {message}")
        except Exception as e:
            self.get_logger().error("Failed to send TCP message: " + str(e))

    # ---------------------------
    #   TCP 입력 수신 스레드
    # ---------------------------
    def tcp_input_thread(self):
        """
        외부 TCP 서버에서 경로 정보를 받습니다.
        예시: 클라이언트가 [1,2,3,4] 같은 JSON 배열을 전송하면, 그 값을 경로로 설정합니다.
        """
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('0.0.0.0', EXTERNAL_PATH_PORT))
        server_socket.listen(5)
        self.get_logger().info(f"Path TCP server listening on port {EXTERNAL_PATH_PORT}")
        while rclpy.ok():
            try:
                client_socket, addr = server_socket.accept()
                self.get_logger().info(f"Received path connection from {addr}")
                data = client_socket.recv(1024)
                if data:
                    try:
                        path_data = json.loads(data.decode('utf-8'))
                        if isinstance(path_data, list):
                            self.set_path(path_data)
                            self.get_logger().info(f"Path updated: {path_data}")
                        else:
                            self.get_logger().warn("Received path data is not a list")
                    except Exception as e:
                        self.get_logger().error("Error parsing path data: " + str(e))
                client_socket.close()
            except Exception as e:
                self.get_logger().error("TCP input thread error: " + str(e))

    # ---------------------------
    #   SUBSCRIPTIONS
    # ---------------------------
    def state_callback(self, msg: String):
        new_state = msg.data.strip().lower()
        valid_states = ['free', 'warn', 'stop']
        if new_state not in valid_states:
            self.get_logger().warn(f"Invalid external state: {new_state}")
            return
        self.get_logger().info(f"External state changed to: {new_state}")
        if new_state == 'free':
            self.desired_speed_factor = 1.0
        elif new_state == 'warn':
            self.desired_speed_factor = 0.5
        elif new_state == 'stop':
            self.desired_speed_factor = 0.0
            self.vx = 0.0
            self.vyaw = 0.0
        self.external_state = new_state

    def pose_callback(self, msg: TransformStamped):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        qz = msg.transform.rotation.z
        qw = msg.transform.rotation.w
        yaw = math.atan2(2.0 * qz * qw, 1.0 - 2.0 * qz * qz)
        self.robot_pose = {'x': x, 'y': y, 'yaw': yaw}

    # ---------------------------
    #   PATH HANDLING
    # ---------------------------
    def set_path(self, node_ids):
        for i in range(len(node_ids) - 1):
            if not self.is_connected(node_ids[i], node_ids[i + 1]):
                self.get_logger().warn(f"Edge not connected: {node_ids[i]} → {node_ids[i + 1]}")
                return
        self.path = node_ids
        self.current_index = 0
        self.state = 'rotate'
        self.get_logger().info(f"Path set: {self.path}")

    def get_node_by_id(self, node_id):
        return next((n for n in self.config['nodes'] if n['id'] == node_id), None)

    def is_connected(self, start_id, end_id):
        for edge in self.config.get('edges', []):
            if ((edge['start'] == start_id and edge['end'] == end_id) or
                (edge['start'] == end_id and edge['end'] == start_id)):
                return True
        return False

    # ---------------------------
    #   MAIN CONTROL LOOP
    # ---------------------------
    def control_loop(self):
        dt = 0.05  # 대략 20Hz 주기

        # 속도 계수 스무딩
        max_sf_change = 0.5 * dt
        sf_error = self.desired_speed_factor - self.current_speed_factor
        if abs(sf_error) < max_sf_change:
            self.current_speed_factor = self.desired_speed_factor
        else:
            self.current_speed_factor += math.copysign(max_sf_change, sf_error)

        if self.robot_pose is None:
            if self.external_state == 'stop':
                self.vel_pub.publish(Twist())
            return
        if self.external_state == 'stop':
            self.vel_pub.publish(Twist())
            return

        # 최종 회전 상태 처리: 완료되면 외부 TCP 서버에 완료 메시지 전송
        if self.state == 'final_rotate':
            cmd = Twist()
            finished_final = self._rotate_in_place(dt)
            cmd.angular.z = self.vyaw * self.current_speed_factor
            self.vel_pub.publish(cmd)
            if finished_final:
                self.state = 'idle'
                self.vx = 0.0
                self.vyaw = 0.0
                self.vel_pub.publish(Twist())
                self.send_tcp_message("FINAL_ORIENTATION_REACHED")
            return

        if len(self.path) < 2:
            if self.external_state == 'stop':
                self.vel_pub.publish(Twist())
            return

        if self.current_index >= len(self.path) - 1:
            self.state = 'idle'
            self.vx = 0.0
            self.vyaw = 0.0
            self.vel_pub.publish(Twist())
            return

        max_speed     = self.driver_params["max_speed"]
        dist_tol      = self.driver_params["dist_tol"]
        kp_line       = self.driver_params["kp_line"]
        kd_line       = self.driver_params["kd_line"]
        kp_heading    = self.driver_params["kp_heading"]
        kd_heading    = self.driver_params["kd_heading"]
        accel         = self.driver_params["accel"]
        yaw_accel     = self.driver_params["yaw_accel"]
        lateral_limit = self.driver_params["lateral_limit"]

        start_node = self.get_node_by_id(self.path[self.current_index])
        target_node = self.get_node_by_id(self.path[self.current_index + 1])
        if not start_node or not target_node:
            self.get_logger().error("Invalid node in path. Stopping.")
            self.state = 'idle'
            self.vel_pub.publish(Twist())
            return

        should_slow_down = self._check_corner_slowdown()
        x0, y0 = start_node['x'], start_node['y']
        x1, y1 = target_node['x'], target_node['y']
        dx, dy = x1 - x0, y1 - y0
        desired_yaw = math.atan2(dy, dx)
        xp, yp = self.robot_pose['x'], self.robot_pose['y']
        dist_to_target = math.hypot(x1 - xp, y1 - yp)
        dot = (xp - x0)*dx + (yp - y0)*dy
        edge_len_sq = dx*dx + dy*dy
        lateral_error = self.compute_lateral_error(x0, y0, x1, y1)

        if self.state != 'recovery' and abs(lateral_error) > lateral_limit:
            self.get_logger().info("Recovery mode: robot too far from edge, returning")
            self.state = 'recovery'
            self.recovery_step = 0

        cmd = Twist()
        if self.state == 'idle':
            self.vx = 0.0
            self.vyaw = 0.0
        elif self.state == 'rotate':
            finished_rotation = self._rotate_in_place(dt, target_yaw=desired_yaw)
            cmd.angular.z = self.vyaw
            if finished_rotation:
                self.state = 'move'
                self.vyaw = 0.0
        elif self.state == 'move':
            if dist_to_target < dist_tol or dot >= edge_len_sq:
                self._advance_path()
            else:
                DECEL_DISTANCE = 0.5
                LATERAL_THRESHOLD = 0.3
                lat_factor = max(
                    0.1,
                    1.0 - min(abs(lateral_error), LATERAL_THRESHOLD) / LATERAL_THRESHOLD
                )
                if should_slow_down:
                    factor = min(1.0, dist_to_target / DECEL_DISTANCE)
                    target_vx = max_speed * factor * lat_factor
                else:
                    target_vx = max_speed * lat_factor
                error_vx = target_vx - self.vx
                step_vx = math.copysign(min(accel * dt, abs(error_vx)), error_vx)
                self.vx += step_vx
                cmd.linear.x = self.vx
                derivative_lateral = (lateral_error - self.prev_lateral_error) / dt
                self.prev_lateral_error = lateral_error
                heading_err = self._angle_diff(desired_yaw, self.robot_pose['yaw'])
                derivative_heading = (heading_err - self.prev_heading_err) / dt
                self.prev_heading_err = heading_err
                u_lat = kp_line * lateral_error + kd_line * derivative_lateral
                u_hdg = kp_heading * heading_err + kd_heading * derivative_heading
                target_vyaw = u_lat + u_hdg
                max_yaw_speed = self.driver_params["max_yaw_speed"]
                target_vyaw = max(-max_yaw_speed, min(max_yaw_speed, target_vyaw))
                error_vyaw = target_vyaw - self.vyaw
                step_vyaw = math.copysign(min(yaw_accel * dt, abs(error_vyaw)), error_vyaw)
                self.vyaw += step_vyaw
                cmd.angular.z = self.vyaw
        elif self.state == 'recovery':
            self._do_recovery(x0, y0, x1, y1, desired_yaw, lateral_error, cmd, dt)

        cmd.linear.x *= self.current_speed_factor
        cmd.angular.z *= self.current_speed_factor
        self.vel_pub.publish(cmd)

    def _rotate_in_place(self, dt, target_yaw=None):
        yaw_tol       = self.driver_params["yaw_tol"]
        kp_yaw        = self.driver_params["kp_yaw"]
        kd_yaw        = self.driver_params["kd_yaw"]
        max_yaw_speed = self.driver_params["max_yaw_speed"]
        yaw_accel     = self.driver_params["yaw_accel"]

        if target_yaw is None:
            target_yaw = self.final_yaw

        yaw_err = self._angle_diff(target_yaw, self.robot_pose['yaw'])
        if abs(yaw_err) < yaw_tol:
            return True

        yaw_diff = yaw_err - self.prev_yaw_err
        yaw_rate_derivative = yaw_diff / dt
        self.prev_yaw_err = yaw_err
        YAW_SPEED_SCALING = 2.0
        scaled_max_yaw = min(max_yaw_speed, abs(yaw_err) * YAW_SPEED_SCALING)
        target_vyaw = kp_yaw * yaw_err + kd_yaw * yaw_rate_derivative
        target_vyaw = max(-scaled_max_yaw, min(scaled_max_yaw, target_vyaw))
        error_vyaw = target_vyaw - self.vyaw
        step_vyaw = math.copysign(min(yaw_accel * dt, abs(error_vyaw)), error_vyaw)
        self.vyaw += step_vyaw
        return False

    def _advance_path(self):
        self.get_logger().info(f"Reached node {self.path[self.current_index + 1]}")
        self.current_index += 1
        if self.current_index >= len(self.path) - 1:
            final_node = self.get_node_by_id(self.path[-1])
            self.final_yaw = final_node.get('theta', 0.0)
            self.get_logger().info(f"Moving to final_rotate state. Target yaw: {self.final_yaw:.3f} rad")
            self.state = 'final_rotate'
            self.vx = 0.0
            self.vyaw = 0.0
            return
        prev_node = self.get_node_by_id(self.path[self.current_index - 1]) if self.current_index > 0 else self.get_node_by_id(self.path[self.current_index])
        curr_node = self.get_node_by_id(self.path[self.current_index])
        next_node = self.get_node_by_id(self.path[self.current_index + 1])
        dx1 = curr_node['x'] - prev_node['x']
        dy1 = curr_node['y'] - prev_node['y']
        dx2 = next_node['x'] - curr_node['x']
        dy2 = next_node['y'] - curr_node['y']
        angle1 = math.atan2(dy1, dx1)
        angle2 = math.atan2(dy2, dx2)
        angle_diff = self._angle_diff(angle2, angle1)
        if abs(angle_diff) > math.radians(30):
            self.get_logger().info(f"Angle change {math.degrees(angle_diff):.1f}° → rotating")
            self.state = 'rotate'
        else:
            self.get_logger().info(f"Angle change {math.degrees(angle_diff):.1f}° → continue moving")
            self.state = 'move'

    def _check_corner_slowdown(self):
        if self.current_index + 2 < len(self.path):
            start_node = self.get_node_by_id(self.path[self.current_index])
            target_node = self.get_node_by_id(self.path[self.current_index + 1])
            next_node = self.get_node_by_id(self.path[self.current_index + 2])
            if not (start_node and target_node and next_node):
                return True
            dx1 = target_node['x'] - start_node['x']
            dy1 = target_node['y'] - start_node['y']
            dx2 = next_node['x'] - target_node['x']
            dy2 = next_node['y'] - target_node['y']
            angle1 = math.atan2(dy1, dx1)
            angle2 = math.atan2(dy2, dx2)
            angle_diff = self._angle_diff(angle2, angle1)
            return abs(angle_diff) > math.radians(15)
        return True

    def _do_recovery(self, x0, y0, x1, y1, desired_yaw, lateral_error, cmd, dt):
        accel         = self.driver_params["accel"]
        yaw_accel     = self.driver_params["yaw_accel"]
        max_yaw_speed = self.driver_params["max_yaw_speed"]
        kp_yaw        = self.driver_params["kp_yaw"]
        kd_yaw        = self.driver_params["kd_yaw"]
        lateral_limit = self.driver_params["lateral_limit"]

        xp, yp = self.robot_pose['x'], self.robot_pose['y']
        dx, dy = x1 - x0, y1 - y0
        edge_len_sq = dx * dx + dy * dy
        if edge_len_sq == 0:
            recovery_yaw = desired_yaw
        else:
            t = ((xp - x0) * dx + (yp - y0) * dy) / edge_len_sq
            t = max(0.0, min(1.0, t))
            proj_x = x0 + t * dx
            proj_y = y0 + t * dy
            to_path_x = proj_x - xp
            to_path_y = proj_y - yp
            recovery_yaw = math.atan2(to_path_y, to_path_x)

        if self.recovery_step == 0:
            finished_rotation = self._rotate_in_place(dt, target_yaw=recovery_yaw)
            cmd.angular.z = self.vyaw
            if finished_rotation:
                target_vx = 0.1
                error_vx = target_vx - self.vx
                step_vx = math.copysign(min(accel * dt, abs(error_vx)), error_vx)
                self.vx += step_vx
                cmd.linear.x = self.vx
                if abs(lateral_error) < lateral_limit * 0.3:
                    self.get_logger().info("Recovery step 1: reached path")
                    self.recovery_step = 1
        elif self.recovery_step == 1:
            finished_rotation = self._rotate_in_place(dt, target_yaw=desired_yaw)
            cmd.angular.z = self.vyaw
            if finished_rotation:
                self.get_logger().info("Recovery complete. Resuming move.")
                self.state = 'move'
                self.recovery_step = 0
                self.vyaw = 0.0

    def compute_lateral_error(self, x0, y0, x1, y1):
        xp, yp = self.robot_pose['x'], self.robot_pose['y']
        dx = x1 - x0
        dy = y1 - y0
        length = math.hypot(dx, dy)
        if length == 0.0:
            return 0.0
        return ((xp - x0) * dy - (yp - y0) * dx) / length

    def _angle_diff(self, a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d


def main(args=None):
    rclpy.init(args=args)
    node = EdgeVelDriver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
