#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import socket
import math
import threading
import time
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped

# 메시지 타입 코드 (예시; 실제 ioServer와 합의된 타입 사용)
PUSH_MSG_TYPE = 0x1000  # 로봇 상태/위치 데이터를 전송할 때 사용할 메시지 타입

class RobotDataPublisher(Node):
    def __init__(self):
        super().__init__('robot_data_publisher')
        # 로봇 상태와 위치 업데이트 구독
        self.create_subscription(String, 'operation_state_info', self.status_callback, 10)
        self.create_subscription(TransformStamped, 'current_pose', self.pose_callback, 10)
        #self.create_subscription(String, 'task/status', self.task_status_callback, 10)

        self.robot_status = "unknown"
        self.current_pose = None  # {"x": ..., "y": ..., "yaw": ...}

        self.task_status = None   # 새 변수: task 상태 정보를 저장
       
        # 시리얼 번호 초기화 (16비트)
        self.serial_counter = 1

        # 기존 push용 TCP 서버 포트 (19301)
        self.push_listen_port = 19301
        self.start_push_server()

        # 새로 task 명령 수신용 TCP 서버 포트 (NAV_PORT: 19206)
        self.task_listen_port = 19206
        self.task_command_pub = self.create_publisher(String, 'task_command', 10)
        self.server_task_msg_pub = self.create_publisher(String, '/server_task_msg', 10)

        self.start_task_server()

    def status_callback(self, msg: String):
        self.robot_status = msg.data

    def pose_callback(self, msg: TransformStamped):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        qz = msg.transform.rotation.z
        qw = msg.transform.rotation.w
        yaw = math.atan2(2.0 * qz * qw, 1.0 - 2.0 * qz * qz)
        self.current_pose = {"x": x, "y": y, "yaw": yaw}
    
    def task_status_callback(self, msg: String):
        data = msg.data.strip().lower()
        if data == "completed":
            self.task_status = 4
        elif data == "running":
            self.task_status = 2
        else:
            try:
                self.task_status = int(data)
            except ValueError:
                self.task_status = None
        self.get_logger().info(f"Updated task_status: {self.task_status}")

    def build_packet(self, msg_type: int, payload: dict) -> bytes:
        body = json.dumps(payload).encode('utf-8')
        header = bytearray(16)
        header[0] = 0x5A
        header[1] = 0x01
        serial = self.serial_counter
        self.serial_counter = (self.serial_counter + 1) & 0xFFFF
        header[2:4] = serial.to_bytes(2, byteorder='big')
        header[4:8] = len(body).to_bytes(4, byteorder='big')
        header[8:10] = msg_type.to_bytes(2, byteorder='big')
        return header + body

    def recv_n_bytes(self, sock: socket.socket, n: int) -> bytes:
        data = bytearray()
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                break
            data.extend(packet)
        return bytes(data)

    def recv_packet(self, sock: socket.socket) -> dict:
        header = self.recv_n_bytes(sock, 16)
        if not header or len(header) < 16:
            return None
        if header[0] != 0x5A or header[1] != 0x01:
            self.get_logger().error("Invalid packet header received.")
            return None
        body_len = int.from_bytes(header[4:8], byteorder='big')
        body = self.recv_n_bytes(sock, body_len)
        try:
            return json.loads(body.decode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Error parsing JSON payload: {e}")
            return None

    # --- 기존 Push 서버 (로봇 상태/위치 전송용) ---
    def start_push_server(self):
        thread = threading.Thread(target=self.tcp_push_server_thread, daemon=True)
        thread.start()

    def tcp_push_server_thread(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('', self.push_listen_port))
        server_sock.listen(5)
        self.get_logger().info(f"Push TCP server listening on port {self.push_listen_port}")
        try:
            while rclpy.ok():
                try:
                    client_sock, addr = server_sock.accept()
                    threading.Thread(
                        target=self.handle_push_connection,
                        args=(client_sock, addr),
                        daemon=True
                    ).start()
                except Exception as e:
                    self.get_logger().error(f"Push accept error: {e}")
                    try:
                        client_sock.close()
                    except:
                        pass
        except Exception as e:
            self.get_logger().error(f"Push TCP server error: {e}")
        finally:
            self.get_logger().info("Closing Push TCP server socket")
            server_sock.close()

    def handle_push_connection(self, sock: socket.socket, addr):
        sock.settimeout(0.5)
        try:
            while rclpy.ok():
                try:
                    if self.current_pose is not None:
                        payload = {
                            "robot_id": "ELLO_01",
                            "status": self.robot_status,
                            "position": self.current_pose,
                            "task_status": self.task_status
                        }
                        packet = self.build_packet(PUSH_MSG_TYPE, payload)
                        sock.sendall(packet)
                    else:
                        self.get_logger().warn("Push: Pose data not yet received; skipping transmission.")
                    time.sleep(0.3)
                except Exception as e:
                    self.get_logger().error(f"Push connection error with {addr}: {e}")
                    break
        finally:
            sock.close()

    # --- 새 Task 명령 수신 서버 (NAV_PORT: 19206) ---
    def start_task_server(self):
        thread = threading.Thread(target=self.tcp_task_server_thread, daemon=True)
        thread.start()

    def tcp_task_server_thread(self):
        task_server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        task_server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        task_server_sock.bind(('', self.task_listen_port))
        task_server_sock.listen(5)
        self.get_logger().info(f"Task TCP server listening on port {self.task_listen_port}")
        try:
            while rclpy.ok():
                try:
                    client_sock, addr = task_server_sock.accept()
                    self.get_logger().info(f"Accepted task connection from {addr}")
                    threading.Thread(
                        target=self.handle_task_connection,
                        args=(client_sock, addr),
                        daemon=True
                    ).start()
                except Exception as e:
                    self.get_logger().error(f"Task accept error: {e}")
                    try:
                        client_sock.close()
                    except:
                        pass
        except Exception as e:
            self.get_logger().error(f"Task TCP server error: {e}")
        finally:
            self.get_logger().info("Closing Task TCP server socket")
            task_server_sock.close()

    def handle_task_connection(self, sock: socket.socket, addr):
        sock.settimeout(5.0)
        try:
            task_command = self.recv_packet(sock)
            if task_command:
                self.get_logger().info(f"Received task command: {task_command}")
                task_msg = String()
                task_msg.data = json.dumps(task_command)
                self.task_command_pub.publish(task_msg)
                self.get_logger().info(f"Published task command on ROS topic: {task_msg.data}")
                if isinstance(task_command, dict) and 'method' in task_command:
                    method_value = task_command['method'].strip().lower()
                    if method_value in ["complete", "pause", "resume", "cancel"]:
                        server_msg = String()
                        server_msg.data = method_value
                        self.server_task_msg_pub.publish(server_msg)
                        self.get_logger().info(f"Published server task message on /server_task_msg: {server_msg.data}")
            else:
                self.get_logger().warn("No valid task command received.")
        except Exception as e:
            self.get_logger().error(f"Error handling task connection from {addr}: {e}")
        finally:
            sock.close()


def main(args=None):
    rclpy.init(args=args)
    node = RobotDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt caught. Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
