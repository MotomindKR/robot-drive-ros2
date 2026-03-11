import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped
import math
import json
import threading

MAX_SPEED = 0.5         # m/s
MAX_YAW_SPEED = 0.8     # rad/s

DIST_TOL = 0.05         # Arrival distance
YAW_TOL  = 0.05         # Alignment angle

# PD Gains for example
KP_YAW  = 3.0
KD_YAW  = 2.9
KP_LINE = 4.0
KD_LINE = 10.2

ACCEL = 0.3         # m/s²
YAW_ACCEL = 2.8     # rad/s²

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

        self.config_path = '/home/gyeum/turtle_ws/src/od2_node_manager/config/config.json'
        self.load_config()

        self.robot_pose = None
        self.start_node = None
        self.target_node = None
        self.state = 'idle'
        self.vx = 0.0
        self.vyaw = 0.0
        self.prev_yaw_err = 0.0
        self.prev_lateral_error = 0.0

        self.external_state = 'free'

        self.timer = self.create_timer(0.05, self.control_loop)
        threading.Thread(target=self.input_thread, daemon=True).start()

    def load_config(self):
        with open(self.config_path) as f:
            self.config = json.load(f)

    def state_callback(self, msg: String):
        new_state = msg.data.strip().lower()
        valid_states = ['free', 'warn', 'stop']
        if new_state not in valid_states:
            self.get_logger().warn(f"Invalid external state: {new_state}")
            return

        if self.external_state == 'stop' and new_state in ['free', 'warn']:
            self.get_logger().info(f"Resuming from stop → {new_state}, resetting velocities.")
            self.vx = 0.0
            self.vyaw = 0.0

        self.external_state = new_state
        self.get_logger().info(f"External state changed to: {new_state}")

    def pose_callback(self, msg: TransformStamped):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        qz = msg.transform.rotation.z
        qw = msg.transform.rotation.w
        yaw = math.atan2(2.0 * qz * qw, 1.0 - 2.0 * qz * qz)
        self.robot_pose = {'x': x, 'y': y, 'yaw': yaw}

    def input_thread(self):
        while True:
            user_input = input("Enter target node ID: ")
            try:
                target_id = int(user_input)
                self.try_drive_to_node(target_id)
            except ValueError:
                print("Invalid input. Please enter an integer node ID.")

    def get_node_by_id(self, node_id):
        return next((n for n in self.config['nodes'] if n['id'] == node_id), None)

    def get_closest_node(self):
        if not self.robot_pose:
            return None
        return min(
            self.config['nodes'],
            key=lambda n: math.hypot(self.robot_pose['x'] - n['x'], self.robot_pose['y'] - n['y'])
        )

    def is_connected(self, start_id, end_id):
        for edge in self.config.get('edges', []):
            if (edge['start'] == start_id and edge['end'] == end_id) or \
               (edge['start'] == end_id and edge['end'] == start_id):
                return True
        return False

    def try_drive_to_node(self, target_id):
        if not self.robot_pose:
            print("Waiting for robot pose...")
            return
        self.start_node = self.get_closest_node()
        self.target_node = self.get_node_by_id(target_id)

        if not self.target_node:
            print("Target node not found.")
            return
        if not self.is_connected(self.start_node['id'], self.target_node['id']):
            print("Nodes are not directly connected by an edge.")
            return

        print(f"Driving from {self.start_node['id']} to {self.target_node['id']}")
        self.state = 'rotate'
        self.prev_yaw_err = 0.0

    def control_loop(self):
        if self.robot_pose is None or self.target_node is None:
            if self.external_state == 'stop':
                self.vel_pub.publish(Twist())
            return

        if self.external_state == 'stop':
            self.vel_pub.publish(Twist())
            return

        speed_factor = 1.0 if self.external_state == 'free' else 0.5

        dx = self.target_node['x'] - self.robot_pose['x']
        dy = self.target_node['y'] - self.robot_pose['y']
        dist = math.hypot(dx, dy)

        desired_yaw = math.atan2(dy, dx)
        yaw_err = self._angle_diff(desired_yaw, self.robot_pose['yaw'])

        cmd = Twist()
        dt = 0.05  # 50 ms

        if self.state == 'idle':
            self.vx = 0.0
            self.vyaw = 0.0

        elif self.state == 'rotate':
            if abs(yaw_err) < YAW_TOL:
                self.state = 'move'
                self.vyaw = 0.0
            else:
                yaw_diff = yaw_err - self.prev_yaw_err
                yaw_rate_derivative = yaw_diff / dt

                # --- 🟡 Add yaw speed scaling by angle error
                YAW_SPEED_SCALING = 2.0  # Tune this value
                scaled_max_yaw = min(MAX_YAW_SPEED, abs(yaw_err) * YAW_SPEED_SCALING)

                target_vyaw = KP_YAW * yaw_err + KD_YAW * yaw_rate_derivative
                target_vyaw = max(-scaled_max_yaw, min(scaled_max_yaw, target_vyaw))

                error_vyaw = target_vyaw - self.vyaw
                step_vyaw = math.copysign(min(YAW_ACCEL * dt, abs(error_vyaw)), error_vyaw)
                self.vyaw += step_vyaw

                cmd.angular.z = self.vyaw * speed_factor
                self.prev_yaw_err = yaw_err


        elif self.state == 'move':
            if dist < DIST_TOL:
                print("Arrived at target.")
                self.state = 'idle'
                self.target_node = None
                self.start_node = None
                self.vx = 0.0
                self.vyaw = 0.0
            else:
                if abs(yaw_err) > 0.3:
                    self.state = 'rotate'
                    self.prev_yaw_err = yaw_err
                else:
                    # 🔽 Speed down near goal
                    SPEED_SCALING = 0.7  # tune this
                    target_vx = min(MAX_SPEED, dist * SPEED_SCALING)

                    error_vx = target_vx - self.vx
                    step_vx = math.copysign(min(ACCEL * dt, abs(error_vx)), error_vx)
                    self.vx += step_vx
                    cmd.linear.x = self.vx * speed_factor

                    # Lateral error for heading control
                    lateral_error = self.compute_lateral_error()
                    d_lateral = (lateral_error - self.prev_lateral_error) / dt
                    self.prev_lateral_error = lateral_error

                    target_vyaw = KP_LINE * lateral_error + KD_LINE * d_lateral
                    target_vyaw = max(-MAX_YAW_SPEED, min(MAX_YAW_SPEED, target_vyaw))

                    error_vyaw = target_vyaw - self.vyaw
                    step_vyaw = math.copysign(min(YAW_ACCEL * dt, abs(error_vyaw)), error_vyaw)
                    self.vyaw += step_vyaw
                    cmd.angular.z = self.vyaw * speed_factor


        self.vel_pub.publish(cmd)

    def compute_lateral_error(self):
        if not self.start_node or not self.target_node:
            return 0.0

        x0, y0 = self.start_node['x'], self.start_node['y']
        x1, y1 = self.target_node['x'], self.target_node['y']
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

