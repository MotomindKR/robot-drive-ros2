#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PUB_HZ = 1.0  # 1 Hz publish

class MapNamePublisher(Node):
    def __init__(self):
        super().__init__('map_name_publisher')
        self.declare_parameter('map_name', 'map_not_loaded')
        self.map_name = self.get_parameter('map_name').value

        # 기본 QoS (depth=10)
        self.pub = self.create_publisher(String, 'map_name', 10)
        self.create_timer(1.0 / PUB_HZ, self.publish_loop)

        self.get_logger().info(f"Publishing map_name='{self.map_name}' at {PUB_HZ} Hz")

    def publish_loop(self):
        msg = String()
        msg.data = self.map_name
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapNamePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
