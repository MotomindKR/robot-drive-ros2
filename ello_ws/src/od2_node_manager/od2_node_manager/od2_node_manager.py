import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json
import math
import os
from geometry_msgs.msg import TransformStamped
import sys


class NodeEdgeVisualizer(Node):
    def __init__(self):
        super().__init__('node_edge_visualizer')
        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)


        json_file_name = self.declare_parameter('mapname', 'home.json').get_parameter_value().string_value
        base_config_path = '/home/nvidia/ello_ws/src/od2_node_manager/config'
        self.config_path = os.path.join(base_config_path, json_file_name)
        self.get_logger().info(f"Loading map from: {self.config_path}")
        
        self.load_config()
        self.last_mtime = os.path.getmtime(self.config_path)
        self.timer = self.create_timer(0.1, self.publish_markers)  # 10 Hz
        self.subscription = self.create_subscription(
            TransformStamped,
            'current_pose',
            self.pose_callback,
            10
        )
        self.robot_x = None
        self.robot_y = None
        self.robot_z = None

    def load_config(self):
        with open(self.config_path) as f:
            self.config = json.load(f)

    def pose_callback(self, msg: TransformStamped):
        self.robot_x = msg.transform.translation.x
        self.robot_y = msg.transform.translation.y
        self.robot_z = msg.transform.translation.z

    def publish_markers(self):
        self.check_for_changes()

        if self.robot_x is None or self.robot_y is None or self.robot_z is None:
            self.get_logger().warn('Robot position not yet received.')
            return

        marker_array = MarkerArray()

        for i, node in enumerate(self.config['nodes']):
            # Create an arrow marker for the node
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.id = i * 2 + 1  # Assign a unique ID to each marker
            marker.pose.position.x = node['x']
            marker.pose.position.y = node['y']
            marker.pose.position.z = node['z']
            marker.pose.orientation.z = math.sin(node['theta'] / 2.0)
            marker.pose.orientation.w = math.cos(node['theta'] / 2.0)
            marker.scale.x = 0.2  # Length of the arrow
            marker.scale.y = 0.05  # Width of the arrow shaft
            marker.scale.z = 0.05  # Height of the arrow shaft
            marker.color.a = 1.0

            distance = math.sqrt((self.robot_x - node['x'])**2 + (self.robot_y - node['y'])**2 + (self.robot_z - node['z'])**2)
            if distance < 0.25:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            marker_array.markers.append(marker)

            # Create a text marker for the node ID
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.id = i * 2 + 2  # Ensure unique ID for text markers
            text_marker.pose.position.x = node['x']
            text_marker.pose.position.y = node['y']
            text_marker.pose.position.z = node['z'] + 0.2  # Slightly above the node marker
            text_marker.scale.z = 0.1  # Height of the text
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = str(node['id'])  # Set the text to the node ID

            marker_array.markers.append(text_marker)

        for j, edge in enumerate(self.config['edges']):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.id = len(self.config['nodes']) * 2 + j + 1  # Ensure unique ID for edges
            marker.scale.x = 0.05
            marker.color.a = 1.0

            start_node = next(node for node in self.config['nodes'] if node['id'] == edge['start'])
            end_node = next(node for node in self.config['nodes'] if node['id'] == edge['end'])

            p1 = Point()
            p1.x = start_node['x']
            p1.y = start_node['y']
            p1.z = start_node['z']
            p2 = Point()
            p2.x = end_node['x']
            p2.y = end_node['y']
            p2.z = end_node['z']

            marker.points.append(p1)
            marker.points.append(p2)

            if self.point_on_line_segment(self.robot_x, self.robot_y, self.robot_z, start_node, end_node):
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.publisher.publish(marker_array)

    def point_on_line_segment(self, px, py, pz, start_node, end_node):
        # Check if point (px, py, pz) is on the line segment between start_node and end_node
        sx, sy, sz = start_node['x'], start_node['y'], start_node['z']
        ex, ey, ez = end_node['x'], end_node['y'], end_node['z']
        line_mag = math.sqrt((ex - sx)**2 + (ey - sy)**2 + (ez - sz)**2)
        point_mag = math.sqrt((px - sx)**2 + (py - sy)**2 + (pz - sz)**2) + math.sqrt((px - ex)**2 + (py - ey)**2 + (pz - ez)**2)
        return math.isclose(line_mag, point_mag, rel_tol=0.1)

    def check_for_changes(self):
        current_mtime = os.path.getmtime(self.config_path)
        if current_mtime != self.last_mtime:
            self.load_config()
            self.last_mtime = current_mtime
            self.get_logger().info("Configuration file reloaded due to external changes.")

def main(args=None):
    rclpy.init(args=args)
    node_edge_visualizer = NodeEdgeVisualizer()
    rclpy.spin(node_edge_visualizer)
    node_edge_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
