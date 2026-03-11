import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class LidarVirtualBumper(Node):
    def __init__(self):
        super().__init__('lidar_virtual_bumper')

        # Publisher for robot state ("free", "warn", "stop")
        self.mb_state_pub = self.create_publisher(String, 'move_bumper_state', 10)
        self.rb_state_pub = self.create_publisher(String, 'rotate_bumper_state', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan_filtered',
            self.scan_callback,
            10
        )

        # Rectangle A (inner, danger zone) — triggers STOP
        self.a_x_min = -0.45
        self.a_x_max = 0.12
        self.a_y_min = -0.28 # real y = -0.26 (2cm margin)
        self.a_y_max = 0.28
        self.a_threshold = 3

        # Rectangle B (outer, caution zone) — triggers WARN
        self.b_x_min = -0.8
        self.b_x_max = 0.12
        self.b_y_min = -0.4
        self.b_y_max = 0.4
        self.b_threshold = 5
        
        # Rectangle C (roate, danger zone) — triggers WARN
        self.c_x_min = -0.2
        self.c_x_max = 0.0
        self.c_y_min = -0.3
        self.c_y_max = 0.3
        self.c_threshold = 3

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("LidarVirtualBumper node running...")

    def is_in_rect(self, x, y, x_min, x_max, y_min, y_max):
        return x_min <= x <= x_max and y_min <= y <= y_max

    def scan_callback(self, scan_msg: LaserScan):
        """
        Count how many points fall into:
        - Rectangle A → stop
        - Rectangle B → warn (only if not enough for stop)
        Otherwise → free
        """
        a_count = 0
        b_count = 0
        c_count = 0

        for i, r in enumerate(scan_msg.ranges):
            if scan_msg.range_min < r < scan_msg.range_max and not math.isinf(r):
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                if self.is_in_rect(x, y, self.a_x_min, self.a_x_max, self.a_y_min, self.a_y_max):
                    a_count += 1
                elif self.is_in_rect(x, y, self.b_x_min, self.b_x_max, self.b_y_min, self.b_y_max):
                    b_count += 1
                elif self.is_in_rect(x, y, self.c_x_min, self.c_x_max, self.c_y_min, self.c_y_max):
                    c_count += 1

        if a_count >= self.a_threshold:
            mbstate = "stop"
        elif b_count >= self.b_threshold:
            mbstate = "warn"
        else:
            mbstate = "free"
	
        if c_count >= self.c_threshold:	
            rbstate = "stop"
        else:
            rbstate = "free"

	
        mbmsg = String()
        rbmsg = String()
        mbmsg.data = mbstate
        rbmsg.data = rbstate
        self.mb_state_pub.publish(mbmsg)
        self.rb_state_pub.publish(rbmsg)

        self.get_logger().info(
            f"Move bumper State: {mbstate}, Rotation bumper State: {rbstate}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarVirtualBumper()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

