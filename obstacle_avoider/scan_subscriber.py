import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if r > 0.0]
        if valid_ranges:
            closest = min(valid_ranges)
            self.get_logger().info(f'Closest obstacle: {closest:.2f} m')
        else:
            self.get_logger().info('No valid readings')

def main(args=None):
    rclpy.init(args=args)
    node = ScanSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
