import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped
import math
import json
import threading

MAX_SPEED = 0.5         # m/s
MAX_YAW_SPEED = 0.5     # rad/s

ACCEL = 0.2            # m/s^2
YAW_ACCEL = 0.2         # rad/s^2

DIST_TOL = 0.03         # Arrival distance
YAW_TOL  = 0.05         # Alignment angle

DECEL_DIST    = 0.3
MIN_FWD_SCALE = 0.0     # minimum fraction of speed

# PD Gains for example
KP_YAW  = 2.0
KD_YAW  = 0.1
KP_LINE = 2.0
KD_LINE = 0.2

class EdgeVelDriver(Node):
    def __init__(self):
        super().__init__('edge_vel_driver')

        # Publishers & Subscribers
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_sub = self.create_subscription(
            TransformStamped, 
            'current_pose', 
            self.pose_callback, 
            10
        )
        # Subscription for external robot_state
        self.state_sub = self.create_subscription(
            String,
            'robot_state',      # <- Set topic name as you wish
            self.state_callback,
            10
        )

        # Load map config (your JSON with nodes & edges)
        self.config_path = '/home/gyeum/robot_arm_ws/src/od2_node_manager/config/config.json'
        self.load_config()

        # Internal states
        self.robot_pose = None
        self.start_node = None
        self.target_node = None
        self.state = 'idle'  # 'idle', 'rotate', or 'move'

        self.vx = 0.0
        self.vyaw = 0.0

        # For derivative control
        self.prev_yaw_err       = 0.0
        self.prev_lateral_error = 0.0

        # External robot state.  "free" by default
        # "free": normal speed
        # "warn": 50% speed
        # "stop": zero speed (freeze)
        self.external_state = 'free'

        # 20 Hz control loop
        self.timer = self.create_timer(0.05, self.control_loop)

        # Input in separate thread
        threading.Thread(target=self.input_thread, daemon=True).start()

    def load_config(self):
        with open(self.config_path) as f:
            self.config = json.load(f)

    def state_callback(self, msg: String):
        """
        Subscriber callback for external robot state:
        'free', 'warn', or 'stop'.
        """
        new_state = msg.data.strip().lower()
        if new_state in ['free','warn','stop']:
            self.external_state = new_state
            self.get_logger().info(f"External state changed to: {new_state}")
        else:
            self.get_logger().warn(f"Invalid external state: {new_state}")

    def pose_callback(self, msg: TransformStamped):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        qz = msg.transform.rotation.z
        qw = msg.transform.rotation.w
        # Convert quaternion -> yaw
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
        self.state        = 'rotate'  # Begin by rotating to face node
        self.vx           = 0.0
        self.vyaw         = 0.0
        self.prev_yaw_err = 0.0

    def control_loop(self):
        """Main control logic, runs at ~20Hz."""
        if self.robot_pose is None or self.target_node is None:
            # If we don't have a target, just keep velocities zero or follow external_state
            if self.external_state == 'stop':
                cmd = Twist()
                self.vel_pub.publish(cmd)
            return

        # -------------------------------
        # 1) Check External Robot State
        # -------------------------------
        if self.external_state == 'stop':
            # "stop" => freeze the robot at 0 velocity.
            # Do NOT advance the internal state machine 
            # so we can resume where we left off later.
            cmd = Twist()
            self.vel_pub.publish(cmd)
            return

        # If we are "free" or "warn", we proceed with normal path logic.
        # Later we’ll multiply speeds by speed_factor (1.0 or 0.5).
        speed_factor = 1.0 if self.external_state == 'free' else 0.5

        # -------------------------------
        # 2) Normal Rotate/Move Logic
        # -------------------------------
        dx = self.target_node['x'] - self.robot_pose['x']
        dy = self.target_node['y'] - self.robot_pose['y']
        dist = math.hypot(dx, dy)

        desired_yaw = math.atan2(dy, dx)
        yaw_err = self._angle_diff(desired_yaw, self.robot_pose['yaw'])

        cmd = Twist()

        if self.state == 'idle':
            self.vx   = 0.0
            self.vyaw = 0.0

        elif self.state == 'rotate':
            # PD for yaw
            if abs(yaw_err) < YAW_TOL:
                self.state = 'move'
                self.vyaw  = 0.0
            else:
                yaw_diff             = yaw_err - self.prev_yaw_err
                yaw_rate_derivative  = yaw_diff / 0.05  # 20Hz -> dt=0.05
                rotate_output        = KP_YAW * yaw_err + KD_YAW * yaw_rate_derivative
                # Saturate
                rotate_output = max(-MAX_YAW_SPEED, min(MAX_YAW_SPEED, rotate_output))
                # Accel-limit
                error_vyaw = rotate_output - self.vyaw
                step_vyaw  = math.copysign(min(YAW_ACCEL*0.05, abs(error_vyaw)), error_vyaw)
                self.vyaw += step_vyaw

                cmd.angular.z = self.vyaw * speed_factor  # Warn => half turn speed
                self.prev_yaw_err = yaw_err

        elif self.state == 'move':
            # Check if we arrived
            if dist < DIST_TOL:
                print("Arrived at target.")
                self.state = 'idle'
                self.target_node = None
                self.start_node  = None
                self.vx   = 0.0
                self.vyaw = 0.0

            else:
                # If yaw error grows big, re-rotate
                if abs(yaw_err) > 0.3:
                    self.state = 'rotate'
                    self.prev_yaw_err = yaw_err
                # Check overshoot
                else:
                    x0, y0 = self.start_node['x'], self.start_node['y']
                    x1, y1 = self.target_node['x'], self.target_node['y']
                    length_sq = (x1 - x0)**2 + (y1 - y0)**2
                    dot = ((self.robot_pose['x'] - x0)*(x1 - x0) + 
                           (self.robot_pose['y'] - y0)*(y1 - y0))
                    if dot > length_sq:
                        print("Overshot node—reorienting.")
                        self.state = 'rotate'
                    else:
                        # Forward speed
                        decel_factor = self._decel_scale(dist, DECEL_DIST)
                        target_vx    = MAX_SPEED * decel_factor
                        # Accel or decel
                        if target_vx < self.vx:
                            self.vx = max(target_vx, self.vx - ACCEL * 0.05)
                        else:
                            self.vx = min(target_vx, self.vx + ACCEL * 0.05)

                        cmd.linear.x = self.vx * speed_factor

                        # Lateral PD
                        lateral_error = self.compute_lateral_error()
                        d_lateral = (lateral_error - self.prev_lateral_error) / 0.05
                        self.prev_lateral_error = lateral_error

                        line_output = KP_LINE * lateral_error + KD_LINE * d_lateral
                        line_output = max(-MAX_YAW_SPEED, min(MAX_YAW_SPEED, line_output))
                        cmd.angular.z = line_output * speed_factor

        # Publish final command
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

        # Signed cross-product distance
        return ((xp - x0) * dy - (yp - y0) * dx) / length

    def _angle_diff(self, a, b):
        d = a - b
        while d > math.pi:
            d -= 2*math.pi
        while d < -math.pi:
            d += 2*math.pi
        return d

    def _decel_scale(self, dist, max_dist):
        scale = dist / max_dist
        return max(MIN_FWD_SCALE, min(1.0, scale))

def main(args=None):
    rclpy.init(args=args)
    node = EdgeVelDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

