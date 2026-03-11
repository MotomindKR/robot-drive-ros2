import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped
import math
import json
from collections import deque
import argparse
import os
import sys
from std_msgs.msg import String


class EdgeVelDriver(Node):
    def __init__(self):
        super().__init__('edge_vel_driver')

        # ───── ROS 인터페이스 ───────────────────────────────────
        self.vel_pub        = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_info_pub = self.create_publisher(String, 'location_info', 10)
        self.op_state_pub   = self.create_publisher(String, 'operation_state_info', 10)
        self.task_status_pub= self.create_publisher(String, 'task_status_info', 10)

        self.pose_sub   = self.create_subscription(TransformStamped, 'current_pose',
                                                   self.pose_callback, 10)
        self.create_subscription(String, 'move_bumper_state',
                                 self.move_state_callback, 10)
        self.create_subscription(String, 'rotate_bumper_state',
                                 self.rotate_state_callback, 10)
        self.create_subscription(String, 'task_command',
                                 self.task_destination_callback, 10)
        self.create_subscription(String, 'server_task_msg',
                                 self.server_task_callback, 10)

        # ───── 설정 파일 경로 ───────────────────────────────────
        json_file_name = self.declare_parameter('mapname', 'home.json')\
                              .get_parameter_value().string_value
        base_cfg_dir   = '/home/nvidia/ello_ws/src/od2_node_manager/config'
        self.config_path        = os.path.join(base_cfg_dir, json_file_name)
        self.driver_config_path = os.path.join(base_cfg_dir, 'edge_driver_config.json')

        # ───── 처음 한 번 로드 ──────────────────────────────────
        self.load_map_config();     self.build_graph()
        self.load_driver_config()

        # ───── 파일 mtime 저장 ─────────────────────────────────
        self.map_mtime    = os.path.getmtime(self.config_path)
        self.driver_mtime = os.path.getmtime(self.driver_config_path)

        # ───── 내부 상태 변수(생략 부분 동일) ────────────────────
        self.robot_pose, self.path, self.current_index = None, [], 0
        self.state = 'idle'
        self.vx, self.vyaw = 0.0, 0.0
        self.prev_yaw_err = self.prev_lateral_error = self.prev_heading_err = 0.0
        self.external_move_state = self.external_rotate_state = 'free'
        self.desired_speed_factor_move = self.current_speed_factor_move = 1.0
        self.desired_speed_factor_rotate = self.current_speed_factor_rotate = 1.0
        self.operation_state, self.task_status = "free", 0
        self.recovery_step, self.final_yaw = 0, 0.0
        self.task_pause = False

        # ───── 주 타이머(제어 루프) & 설정 감시 타이머 ──────────
        self.create_timer(0.05, self.control_loop)     # 20 Hz
        self.create_timer(1.0,  self.check_config_updates)  # 1 Hz

    # ───────────────────────────────────────────────────────────
    # 설정 파일 변경 감지 및 재‑로딩
    # ───────────────────────────────────────────────────────────
    def check_config_updates(self):
        """두 JSON 파일의 mtime을 주기적으로 확인해 바뀌면 즉시 재‑로딩."""
        try:
            new_map_mtime = os.path.getmtime(self.config_path)
            if new_map_mtime != self.map_mtime:
                self.get_logger().info(f"Detected change in {self.config_path} → reloading map")
                self.map_mtime = new_map_mtime
                self.load_map_config()
                self.build_graph()
                # 경로 재계산 필요 시 여기에 추가 로직

            new_drv_mtime = os.path.getmtime(self.driver_config_path)
            if new_drv_mtime != self.driver_mtime:
                self.get_logger().info(f"Detected change in {self.driver_config_path} → reloading driver params")
                self.driver_mtime = new_drv_mtime
                self.load_driver_config()

        except FileNotFoundError as e:
            self.get_logger().error(f"Config file missing while watching: {e}")
        except Exception as e:
            self.get_logger().error(f"Exception in check_config_updates: {e}")

    # ---------------------------
    #   CONFIGURATION LOADING
    # ---------------------------
    def load_map_config(self):
        """Load map configuration from file into self.config."""
        with open(self.config_path) as f:
            self.config = json.load(f)

    def load_driver_config(self):
        """Load driving parameters from file into self.driver_params."""
        with open(self.driver_config_path) as f:
            self.driver_params = json.load(f)

    # ---------------------------
    #   GRAPH BUILDING & PATH FINDING
    # ---------------------------
    def build_graph(self):
        """Build an adjacency list from the map configuration."""
        self.graph = {}
        for node in self.config['nodes']:
            node_id = node['id']
            self.graph[node_id] = []
        for edge in self.config.get('edges', []):
            s = edge['start']
            e = edge['end']
            self.graph[s].append(e)
            self.graph[e].append(s)

    def find_closest_node(self, x, y):
        """Return the node ID closest to (x, y)."""
        best_node_id = None
        best_dist = float('inf')
        for node in self.config['nodes']:
            nx, ny = node['x'], node['y']
            dist = math.hypot(nx - x, ny - y)
            if dist < best_dist:
                best_dist = dist
                best_node_id = node['id']
        return best_node_id

    def compute_path_bfs(self, start_id, goal_id):
        """Compute a BFS path (list of node IDs) from start_id to goal_id."""
        # If start == goal, just return a single-node path (robot is already there).
        if start_id == goal_id:
            return [start_id]

        visited = set([start_id])
        queue = deque([(start_id, [start_id])])
        while queue:
            current, path = queue.popleft()
            if current == goal_id:
                return path
            for neighbor in self.graph[current]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return None

    # ---------------------------
    #   SUBSCRIPTIONS & TASK COMMANDS
    # ---------------------------
    def move_state_callback(self, msg: String):
        """Handle move bumper state messages."""
        new_state = msg.data.strip().lower()
        if new_state not in ['free', 'warn', 'stop']:
            self.get_logger().warn(f"Invalid move state: {new_state}")
            return
        if new_state == 'free':
            self.desired_speed_factor_move = 1.0
        elif new_state == 'warn':
            self.desired_speed_factor_move = 0.5
        elif new_state == 'stop':
            self.desired_speed_factor_move = 0.0
            self.vx = 0.0
        self.external_move_state = new_state

    def rotate_state_callback(self, msg: String):
        """Handle rotate bumper state messages."""
        new_state = msg.data.strip().lower()
        if new_state not in ['free', 'warn', 'stop']:
            self.get_logger().warn(f"Invalid rotate state: {new_state}")
            return
        if new_state == 'free':
            self.desired_speed_factor_rotate = 1.0
        elif new_state == 'warn':
            self.desired_speed_factor_rotate = 0.5
        elif new_state == 'stop':
            self.desired_speed_factor_rotate = 0.0
            self.vyaw = 0.0
        self.external_rotate_state = new_state

    def pose_callback(self, msg: TransformStamped):
        """Update the current robot pose."""
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        qz = msg.transform.rotation.z
        qw = msg.transform.rotation.w
        yaw = math.atan2(2.0 * qz * qw, 1.0 - 2.0 * (qz ** 2))
        self.robot_pose = {'x': x, 'y': y, 'yaw': yaw}

    def task_destination_callback(self, msg: String):
        """
        Callback for task commands on "task_command". Parses the JSON command,
        updates task status, computes a path to the destination, and sets the operation state.
        """
        try:
            # Set task status to RUNNING (2) and publish "running".
            self.task_status = 2
            status_msg = String()
            status_msg.data = "running"
            self.task_status_pub.publish(status_msg)
            self.get_logger().info("Published task status 'running'.")

            task_command = json.loads(msg.data)
            self.get_logger().info(f"[Task] Raw task command received: {msg.data}")

            if isinstance(task_command, dict):
                if "id" in task_command:
                    destination = task_command["id"]
                    self.get_logger().info(f"[Task] Received destination id: {destination}")
                    if isinstance(destination, str):
                        try:
                            destination = int(destination)
                        except ValueError:
                            self.get_logger().error("Destination id is not a valid integer.")
                            return
                else:
                    self.get_logger().warn("[Task] Task command received but no 'id' field. Ignoring command.")
                    return
            elif isinstance(task_command, list):
                destination = task_command
            elif isinstance(task_command, str):
                destination = task_command
            else:
                self.get_logger().error(f"[Task] Unsupported task command type: {type(task_command)}")
                return

            if self.robot_pose is None:
                self.get_logger().error("No robot pose available; cannot compute path.")
                return

            start_id = self.find_closest_node(self.robot_pose["x"], self.robot_pose["y"])
            self.get_logger().info(f"[Task] Nearest node to robot is {start_id}, destination is {destination}")
            path = self.compute_path_bfs(start_id, destination)
            if not path:
                self.get_logger().error(f"[Task] No path found from {start_id} to {destination}!")
            else:
                self.get_logger().info(f"[Task] Path found: {path}")
                self.set_path(path)
                # We only set "operating" if there's actually something to do or align.
                self.operation_state = "operating"
        except Exception as e:
            self.get_logger().error(f"[Task] Exception in task_destination_callback: {e}")

    def server_task_callback(self, msg: String):
        """
        Callback for server task control messages (from "server_task_msg").
        Accepts "complete", "pause", "resume", and "cancel":
          - "pause": Stop the robot (cmd_vel = 0) and set the pause flag.
          - "resume": Clear the pause flag.
          - "cancel" or "complete": Stop the robot, clear the task (erase path),
            set state to idle, and set operation state to "free".
        """
        command = msg.data.strip().lower()
        if command == "pause":
            self.task_pause = True
            self.get_logger().info("Task paused. Stopping the robot.")
            zero_cmd = Twist()
            self.vel_pub.publish(zero_cmd)
        elif command == "resume":
            self.task_pause = False
            self.get_logger().info("Task resumed.")
        elif command in ["cancel", "complete"]:
            self.task_pause = False
            self.path = []
            self.state = 'idle'
            self.operation_state = "free"
            self.get_logger().info("Task canceled/completed. Clearing tasks and stopping the robot.")
            zero_cmd = Twist()
            self.vel_pub.publish(zero_cmd)
        else:
            self.get_logger().warn(f"Unknown server task command: {command}")

    # ---------------------------
    #   PATH HANDLING
    # ---------------------------
    def set_path(self, node_ids):
        """
        Set the planned path (a list of node IDs) and initialize navigation state.
        Handle single-node path as a valid request to rotate in place if needed.
        """
        # First check that edges exist for multi-node paths.
        for i in range(len(node_ids) - 1):
            if not self.is_connected(node_ids[i], node_ids[i + 1]):
                self.get_logger().error(f"Edge not connected: {node_ids[i]} -> {node_ids[i + 1]}")
                return

        self.path = node_ids
        self.current_index = 0

        if len(node_ids) == 1:
            # Robot is already "at" the destination node.
            single_node = self.get_node_by_id(node_ids[0])
            if single_node is not None:
                self.final_yaw = single_node.get('theta', 0.0)
                self.state = 'final_rotate'
                self.get_logger().info(
                    f"Single-node path. Will rotate to theta={self.final_yaw:.3f}, then mark complete."
                )
            else:
                self.state = 'idle'
                self.operation_state = "complete"
                self.get_logger().warn("Single-node path but node is missing in config; marking complete.")
            return
        else:
            self.state = 'rotate'
            self.get_logger().info(f"Path set: {self.path}")

    def get_node_by_id(self, node_id):
        """Return the node dictionary for a given node ID."""
        return next((n for n in self.config['nodes'] if n['id'] == node_id), None)

    def is_connected(self, start_id, end_id):
        """Return True if there is an edge connecting start_id and end_id."""
        for edge in self.config.get('edges', []):
            if ((edge['start'] == start_id and edge['end'] == end_id) or
                (edge['start'] == end_id and edge['end'] == start_id)):
                return True
        return False

    # ---------------------------
    #   MAIN CONTROL LOOP
    # ---------------------------
    def control_loop(self):
        """
        Main control loop (~20 Hz): executes the state machine and publishes commands.
        Also publishes location info and operation state.
        """
        dt = 0.05  # 20 Hz

        # 1) Pause handling
        if self.task_pause:
            self.vel_pub.publish(Twist())
            return

        # 2) Ramp bumper speed factors
        max_sf_change = 0.5 * dt
        # move factor
        sf_error_move = self.desired_speed_factor_move - self.current_speed_factor_move
        if abs(sf_error_move) < max_sf_change:
            self.current_speed_factor_move = self.desired_speed_factor_move
        else:
            self.current_speed_factor_move += math.copysign(max_sf_change, sf_error_move)
        # rotate factor
        sf_error_rotate = self.desired_speed_factor_rotate - self.current_speed_factor_rotate
        if abs(sf_error_rotate) < max_sf_change:
            self.current_speed_factor_rotate = self.desired_speed_factor_rotate
        else:
            self.current_speed_factor_rotate += math.copysign(max_sf_change, sf_error_rotate)

        # 3) Pose validity
        if self.robot_pose is None:
            return

        # 4) If not driving, just publish status
        driving_states = ['rotate', 'move', 'recovery', 'final_rotate']
        if self.state not in driving_states:
            self.publish_location_info()
            self._publish_op_state()
            return

        # 5) Final in-place rotation
        if self.state == 'final_rotate':
            cmd = Twist()
            done = self._rotate_in_place(dt)
            cmd.angular.z = self.vyaw * self.current_speed_factor_rotate
            self.vel_pub.publish(cmd)

            if done:
                # stop and complete
                self.state = 'idle'
                self.vx = 0.0
                self.vyaw = 0.0
                self.vel_pub.publish(Twist())
                self.get_logger().info("Final orientation reached. Stopped.")
                self.operation_state = "complete"

            self.publish_location_info()
            self._publish_op_state()
            return

        # 6) Ensure there's a path to follow
        if len(self.path) < 2:
            self.publish_location_info()
            self._publish_op_state()
            return

        # 7) Completed path
        if self.current_index >= len(self.path) - 1:
            self.state = 'idle'
            self.vx = 0.0
            self.vyaw = 0.0
            self.vel_pub.publish(Twist())
            self.operation_state = "complete"
            self.publish_location_info()
            self._publish_op_state()
            return

        # 8) Load driving parameters
        p = self.driver_params
        max_speed     = p["max_speed"]
        dist_tol      = p["dist_tol"]
        kp_line       = p["kp_line"]
        kd_line       = p["kd_line"]
        kp_heading    = p["kp_heading"]
        kd_heading    = p["kd_heading"]
        accel         = p["accel"]
        yaw_accel     = p["yaw_accel"]
        lateral_limit = p["lateral_limit"]
        max_yaw_speed = p["max_yaw_speed"]

        # 9) Segment geometry
        start_node  = self.get_node_by_id(self.path[self.current_index])
        target_node = self.get_node_by_id(self.path[self.current_index + 1])
        x0, y0      = start_node['x'], start_node['y']
        x1, y1      = target_node['x'], target_node['y']
        dx, dy      = x1 - x0, y1 - y0
        desired_yaw = math.atan2(dy, dx)
        xp, yp      = self.robot_pose['x'], self.robot_pose['y']
        dist_to_target = math.hypot(x1 - xp, y1 - yp)
        dot = (xp - x0) * dx + (yp - y0) * dy
        edge_len_sq   = dx*dx + dy*dy
        lateral_error = self.compute_lateral_error(x0, y0, x1, y1)

        cmd = Twist()

        # 10) State handling
        if self.state == 'rotate':
            done = self._rotate_in_place(dt, target_yaw=desired_yaw)
            cmd.angular.z = self.vyaw * self.current_speed_factor_rotate
            if done:
                self.state = 'move'
                self.vyaw = 0.0

        elif self.state == 'move':
            # If we reach or overshoot the node, step path but keep last vel
            if dist_to_target < dist_tol or dot >= edge_len_sq:
                self._advance_path()
                cmd.linear.x  = self.vx  * self.current_speed_factor_move
                cmd.angular.z = self.vyaw * self.current_speed_factor_rotate

            else:
                # longitudinal PD for speed along path
                DECEL_DIST = 0.5
                LAT_THRES  = 0.3
                lat_factor = max(0.1, 1.0 - min(abs(lateral_error), LAT_THRES)/LAT_THRES)
                if self._check_corner_slowdown():
                    factor = min(1.0, dist_to_target/DECEL_DIST)
                    target_vx = max_speed * factor * lat_factor
                else:
                    target_vx = max_speed * lat_factor
                # accel ramp
                err_vx  = target_vx - self.vx
                step_vx = math.copysign(min(accel*dt, abs(err_vx)), err_vx)
                self.vx += step_vx
                cmd.linear.x = self.vx * self.current_speed_factor_move

                # lateral & heading PD -> yaw rate
                d_lat = (lateral_error - self.prev_lateral_error)/dt
                self.prev_lateral_error = lateral_error
                hdg_err = self._angle_diff(desired_yaw, self.robot_pose['yaw'])
                d_hdg  = (hdg_err - self.prev_heading_err)/dt
                self.prev_heading_err = hdg_err
                u_lat = kp_line    * lateral_error + kd_line    * d_lat
                u_hdg = kp_heading * hdg_err        + kd_heading * d_hdg
                tgt_vyaw = u_lat + u_hdg
                # clip and accel
                tgt_vyaw = max(-max_yaw_speed, min(max_yaw_speed, tgt_vyaw))
                err_vyaw  = tgt_vyaw - self.vyaw
                step_y    = math.copysign(min(yaw_accel*dt, abs(err_vyaw)), err_vyaw)
                self.vyaw += step_y
                cmd.angular.z = self.vyaw * self.current_speed_factor_rotate

        elif self.state == 'recovery':
            self._do_recovery(x0, y0, x1, y1, desired_yaw, lateral_error, cmd, dt)

        # 11) Publish
        self.vel_pub.publish(cmd)
        self.publish_location_info()
        self._publish_op_state()

    def publish_location_info(self):
        """
        Publish location_info as a JSON string list containing:
        [start_node, current_location, goal_node].
        If no path exists, "no_path" is published.
        """
        state_msg = String()
        if not self.path:
            state_msg.data = "no_path"
        else:
            start_node = f"node{self.path[0]}"
            current_node = f"node{self.path[self.current_index]}"
            goal_node = f"node{self.path[-1]}"
            location_list = [start_node, current_node, goal_node]
            state_msg.data = json.dumps(location_list)
        self.state_info_pub.publish(state_msg)

    def _publish_op_state(self):
        """Publish the current operation state ('free', 'operating', or 'complete')."""
        op_msg = String()
        op_msg.data = self.operation_state
        self.op_state_pub.publish(op_msg)

    # ---------------------------
    #   HELPER FUNCTIONS
    # ---------------------------
    def _rotate_in_place(self, dt, target_yaw=None):
        """
        Rotate in place using a PD controller until within tolerance.
        Returns True if the target yaw is achieved.
        """
        yaw_tol = self.driver_params["yaw_tol"]
        kp_yaw = self.driver_params["kp_yaw"]
        kd_yaw = self.driver_params["kd_yaw"]
        max_yaw_speed = self.driver_params["max_yaw_speed"]
        yaw_accel = self.driver_params["yaw_accel"]

        if target_yaw is None:
            target_yaw = self.final_yaw
        yaw_err = self._angle_diff(target_yaw, self.robot_pose['yaw'])
        if abs(yaw_err) < yaw_tol:
            return True

        yaw_diff = yaw_err - self.prev_yaw_err
        yaw_rate_derivative = yaw_diff / dt
        self.prev_yaw_err = yaw_err

        # Scale the maximum possible yaw speed based on how large the yaw error is
        YAW_SPEED_SCALING = 2.0
        scaled_max_yaw = min(max_yaw_speed, abs(yaw_err) * YAW_SPEED_SCALING)
        target_vyaw = kp_yaw * yaw_err + kd_yaw * yaw_rate_derivative
        # Clip to scaled max
        target_vyaw = max(-scaled_max_yaw, min(scaled_max_yaw, target_vyaw))

        # Ramp toward target_vyaw
        error_vyaw = target_vyaw - self.vyaw
        step_vyaw = math.copysign(min(yaw_accel * dt, abs(error_vyaw)), error_vyaw)
        self.vyaw += step_vyaw

        return False

    def _advance_path(self):
        """
        Advance to the next node in the planned path.
        Enter 'final_rotate' state upon reaching the final node.
        """
        self.get_logger().info(f"Reached node {self.path[self.current_index + 1]}")
        self.current_index += 1
        if self.current_index >= len(self.path) - 1:
            final_node = self.get_node_by_id(self.path[-1])
            self.final_yaw = final_node.get('theta', 0.0)
            self.get_logger().info(f"Moving to final_rotate. Target yaw: {self.final_yaw:.3f} rad")
            self.state = 'final_rotate'
            self.vx = 0.0
            self.vyaw = 0.0
            return

        curr_node = self.get_node_by_id(self.path[self.current_index])
        next_node = self.get_node_by_id(self.path[self.current_index + 1])
        prev_node = self.get_node_by_id(self.path[self.current_index - 1]) if self.current_index > 0 else curr_node
        dx1 = curr_node['x'] - prev_node['x']
        dy1 = curr_node['y'] - prev_node['y']
        dx2 = next_node['x'] - curr_node['x']
        dy2 = next_node['y'] - curr_node['y']
        angle1 = math.atan2(dy1, dx1)
        angle2 = math.atan2(dy2, dx2)
        angle_diff = self._angle_diff(angle2, angle1)
        if abs(angle_diff) > math.radians(15):
            self.get_logger().info(f"Angle change {math.degrees(angle_diff):.1f}° → rotating")
            self.state = 'rotate'
        else:
            self.get_logger().info(f"Angle change {math.degrees(angle_diff):.1f}° → continue moving")
            self.state = 'move'

    def _check_corner_slowdown(self):
        """
        Check if a significant turn exists ahead and if so, instruct a slowdown.
        """
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
        """
        Recovery behavior: rotate toward the path and move slowly until lateral error is reduced.
        """
        accel = self.driver_params["accel"]
        yaw_accel = self.driver_params["yaw_accel"]
        lateral_limit = self.driver_params["lateral_limit"]
        xp, yp = self.robot_pose['x'], self.robot_pose['y']
        dx, dy = x1 - x0, y1 - y0
        edge_length_sq = dx * dx + dy * dy
        if edge_length_sq == 0:
            recovery_yaw = desired_yaw
        else:
            t = ((xp - x0) * dx + (yp - y0) * dy) / edge_length_sq
            t = max(0.0, min(1.0, t))
            proj_x = x0 + t * dx
            proj_y = y0 + t * dy
            to_path_x = proj_x - xp
            to_path_y = proj_y - yp
            recovery_yaw = math.atan2(to_path_y, to_path_x)

        if self.recovery_step == 0:
            finished_rotation = self._rotate_in_place(dt, target_yaw=recovery_yaw)
            cmd.angular.z = self.vyaw * self.current_speed_factor_rotate
            if finished_rotation:
                target_vx = 0.1
                error_vx = target_vx - self.vx
                step_vx = math.copysign(min(accel * dt, abs(error_vx)), error_vx)
                self.vx += step_vx
                cmd.linear.x = self.vx * self.current_speed_factor_move
                if abs(lateral_error) < lateral_limit * 0.3:
                    self.get_logger().info("Recovery step 1: reached path")
                    self.recovery_step = 1
        elif self.recovery_step == 1:
            finished_rotation = self._rotate_in_place(dt, target_yaw=desired_yaw)
            cmd.angular.z = self.vyaw * self.current_speed_factor_rotate
            if finished_rotation:
                self.get_logger().info("Recovery complete. Resuming move.")
                self.state = 'move'
                self.recovery_step = 0
                self.vyaw = 0.0

    def compute_lateral_error(self, x0, y0, x1, y1):
        """
        Compute the signed distance from the robot's current position to the line defined by (x0, y0) and (x1, y1).
        """
        xp, yp = self.robot_pose['x'], self.robot_pose['y']
        dx = x1 - x0
        dy = y1 - y0
        length = math.hypot(dx, dy)
        if length == 0.0:
            return 0.0
        return ((xp - x0) * dy - (yp - y0) * dx) / length

    def _angle_diff(self, a, b):
        """Return the difference between angles a and b in the interval [-pi, pi]."""
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

def main(args=None):
    rclpy.init(args=args)
    node = EdgeVelDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
