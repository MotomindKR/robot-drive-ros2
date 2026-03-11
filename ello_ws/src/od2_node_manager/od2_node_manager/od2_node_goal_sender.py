import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import json
import math
import os

class NavigationGoalSender(Node):
    def __init__(self):
        super().__init__('navigation_goal_sender')
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.config_path = '/home/gyeum/turtle_ws/src/od2_node_manager/config/config.json'
        self.load_config()
        self.last_mtime = os.path.getmtime(self.config_path)
        self.timer = self.create_timer(1.0, self.check_for_changes)  # Check for changes every second
        self.read_keyboard_input()

    def load_config(self):
        with open(self.config_path) as f:
            self.config = json.load(f)

    def check_for_changes(self):
        current_mtime = os.path.getmtime(self.config_path)
        if current_mtime != self.last_mtime:
            self.load_config()
            self.last_mtime = current_mtime
            self.get_logger().info("Configuration file reloaded due to external changes.")

    def read_keyboard_input(self):
        while True:
            node_id = input("Enter node ID to send navigation goal: ")
            try:
                node_id = int(node_id)
                self.send_navigation_goal(node_id)
            except ValueError:
                print("Invalid input. Please enter a valid node ID.")

    def send_navigation_goal(self, node_id):
        node = next((node for node in self.config['nodes'] if node['id'] == node_id), None)
        if node:
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = node['x']
            goal.pose.position.y = node['y']
            goal.pose.position.z = node['z']
            goal.pose.orientation.z = math.sin(node['theta'] / 2.0)
            goal.pose.orientation.w = math.cos(node['theta'] / 2.0)
            self.goal_publisher.publish(goal)
            self.get_logger().info(f"Sent navigation goal to node {node_id}")
        else:
            self.get_logger().warn(f"Node ID {node_id} not found")

def main(args=None):
    rclpy.init(args=args)
    navigation_goal_sender = NavigationGoalSender()
    rclpy.spin(navigation_goal_sender)
    navigation_goal_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()