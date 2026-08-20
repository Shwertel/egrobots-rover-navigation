import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider_node')

        # Obstacle avoidance parameters
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('clear_distance', 0.7)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('cone_angle_deg', 30.0)

        # Goal navigation parameters
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('heading_kp', 1.5)

        self.enabled = False
        self.avoiding = False
        self.turn_direction = 1.0
        self.goal_reached = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_odom = False

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.start_service = self.create_service(Trigger, 'start_avoidance', self.start_callback)
        self.stop_service = self.create_service(Trigger, 'stop_avoidance', self.stop_callback)

        self.get_logger().info('Obstacle avoider node ready. Call /start_avoidance to begin.')

    def start_callback(self, request, response):
        self.enabled = True
        self.avoiding = False
        self.goal_reached = False
        response.success = True
        response.message = 'Started'
        return response

    def stop_callback(self, request, response):
        self.enabled = False
        self.publisher.publish(Twist())
        response.success = True
        response.message = 'Stopped'
        return response

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.have_odom = True

    def scan_callback(self, msg):
        if not self.enabled or not self.have_odom or self.goal_reached:
            return

        safe_distance = self.get_parameter('safe_distance').value
        clear_distance = self.get_parameter('clear_distance').value
        linear_speed = self.get_parameter('linear_speed').value
        angular_speed = self.get_parameter('angular_speed').value
        cone_angle_deg = self.get_parameter('cone_angle_deg').value
        goal_x = self.get_parameter('goal_x').value
        goal_y = self.get_parameter('goal_y').value
        goal_tolerance = self.get_parameter('goal_tolerance').value
        heading_kp = self.get_parameter('heading_kp').value

        num_readings = len(msg.ranges)
        if num_readings == 0:
            return

        cone_angle_rad = math.radians(cone_angle_deg)
        center_index = int((0.0 - msg.angle_min) / msg.angle_increment)
        cone_size = int(cone_angle_rad / msg.angle_increment)

        left_ranges = [
            r for r in msg.ranges[center_index:center_index + cone_size]
            if r > 0.0
        ]
        right_ranges = [
            r for r in msg.ranges[max(0, center_index - cone_size):center_index]
            if r > 0.0
        ]
        all_valid = left_ranges + right_ranges
        closest = min(all_valid) if all_valid else float('inf')

        cmd = Twist()
        # --- Priority 1: obstacle avoidance ---
        if not self.avoiding and closest < safe_distance:
            self.avoiding = True
            left_min = min(left_ranges) if left_ranges else float('inf')
            right_min = min(right_ranges) if right_ranges else float('inf')
            self.turn_direction = 1.0 if right_min < left_min else -1.0
            self.get_logger().info(
                f'Obstacle at {closest:.2f} m — avoiding, '
                f'turning {"left" if self.turn_direction > 0 else "right"}'
            )

        if self.avoiding:
            if closest > clear_distance:
                self.avoiding = False
                self.get_logger().info(f'Path clear ({closest:.2f} m) — resuming navigation')
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_direction * angular_speed
                self.publisher.publish(cmd)
                return

        # --- Priority 2: goal navigation (only runs when not avoiding) ---
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance_to_goal = math.hypot(dx, dy)

        if distance_to_goal < goal_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.goal_reached = True
            self.get_logger().info(f'Goal reached at ({self.current_x:.2f}, {self.current_y:.2f})')
        else:
            desired_heading = math.atan2(dy, dx)
            heading_error = desired_heading - self.current_yaw
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

            angular_cmd = heading_kp * heading_error
            angular_cmd = max(-angular_speed, min(angular_speed, angular_cmd))

            cmd.angular.z = angular_cmd
            cmd.linear.x = linear_speed * max(0.0, math.cos(heading_error))

            self.get_logger().info(
                f'Heading to goal: dist={distance_to_goal:.2f} m, '
                f'heading_err={math.degrees(heading_error):.1f} deg'
            )

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
