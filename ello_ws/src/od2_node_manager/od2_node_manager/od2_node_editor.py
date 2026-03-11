import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import json
import math
import os
import random
import time  # Import the time module

class NodeEditor(Node):
    def __init__(self):
        super().__init__('node_editor')
        self.config_path = '/home/gyeum/robot_arm_ws/src/od2_node_manager/config/home.json'
        self.config = self.load_config()
        self.current_pose = None
        self.last_mtime = os.path.getmtime(self.config_path)

        self.subscription = self.create_subscription(
            TransformStamped,
            'current_pose',
            self.pose_callback,
            10
        )

    def load_config(self):
        if os.path.exists(self.config_path):
            with open(self.config_path, 'r') as f:
                return json.load(f)
        else:
            return {"nodes": [], "edges": []}

    def save_config(self):
        with open(self.config_path, 'w') as f:
            json.dump(self.config, f, indent=4)

    def pose_callback(self, msg: TransformStamped):
        self.current_pose = msg
        self.get_logger().info("Updated current_pose")

    def update_current_pose(self):
        rclpy.spin_once(self, timeout_sec=1.0)
        if self.current_pose is None:
            print("Current pose not available. Please ensure the robot is publishing its pose.")
            return False
        return True

    def add_node(self):
        if not self.update_current_pose():
            return

        node_name = input("Enter node name: ").strip()
        node_id = input("Enter node ID: ").strip()

        try:
            node_id = int(node_id)
        except ValueError:
            print("Invalid node ID. Please enter a valid integer.")
            return
        
        print("Wait for 5 seconds to stabilize the robot pose.")
        time.sleep(5)
        position = self.current_pose.transform.translation
        orientation = self.current_pose.transform.rotation
        x, y, z = position.x, position.y, position.z
        theta = 2 * math.atan2(orientation.z, orientation.w)

        # Add a small random offset to the coordinates
        offset = 0.01
        x += random.uniform(-offset, offset)
        y += random.uniform(-offset, offset)
        z += random.uniform(-offset, offset)

        new_node = {
            "id": node_id,
            "name": node_name,
            "x": x,
            "y": y,
            "z": z,
            "theta": theta,
            "type": "default",
            "description": f"This is {node_name}"
        }

        self.config['nodes'].append(new_node)
        self.save_config()
        print(f"Node {node_name} with ID {node_id} added successfully.")


    def delete_node(self):
        if not self.config['nodes']:
            print("No nodes available to delete.")
            return

        print("Available nodes:")
        for node in self.config['nodes']:
            print(f"ID: {node['id']}, Name: {node['name']}")

        node_id = input("Enter the ID of the node to delete: ").strip()

        try:
            node_id = int(node_id)
        except ValueError:
            print("Invalid node ID. Please enter a valid integer.")
            return

        self.config['nodes'] = [node for node in self.config['nodes'] if node['id'] != node_id]
        self.save_config()
        print(f"Node with ID {node_id} deleted successfully.")

    def check_for_changes(self):
        current_mtime = os.path.getmtime(self.config_path)
        if current_mtime != self.last_mtime:
            self.config = self.load_config()
            self.last_mtime = current_mtime
            self.get_logger().info("Configuration file reloaded due to external changes.")

def main(args=None):
    rclpy.init(args=args)
    node_editor = NodeEditor()

    try:
        while rclpy.ok():
            rclpy.spin_once(node_editor, timeout_sec=0.1)
            command = input("Enter command (add/del/exit): ").strip().lower()
            if command == "add":
                node_editor.add_node()
            elif command == "del":
                node_editor.delete_node()
            elif command == "exit":
                break
            else:
                print("Invalid command. Please enter 'add', 'del', or 'exit'.")
    except KeyboardInterrupt:
        pass

    node_editor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
