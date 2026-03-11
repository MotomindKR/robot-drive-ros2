import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.timer_callback)  # Check every 0.5 seconds
        self.pose_publisher = self.create_publisher(TransformStamped, 'current_pose', 10)

    def timer_callback(self):
        try:
            # Lookup the transform from the 'map' frame to 'base_link' frame
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.display_pose(trans)
        except Exception as e:
            self.get_logger().error(f"Could not transform: {e}")

    def display_pose(self, trans: TransformStamped):
        position = trans.transform.translation
        orientation = trans.transform.rotation
        # Log position and orientation with two decimal places
        self.get_logger().info(
            f'Current Pose - Position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}, '
            f'Orientation: x={orientation.x:.2f}, y={orientation.y:.2f}, z={orientation.z:.2f}, w={orientation.w:.2f}'
        )
        self.pose_publisher.publish(trans)
        

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
