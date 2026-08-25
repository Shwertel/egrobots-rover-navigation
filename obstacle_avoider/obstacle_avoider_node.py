import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, PointStamped, TransformStamped, Point
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from tf2_ros import (Buffer, TransformListener, TransformBroadcaster,
                     LookupException, ExtrapolationException, ConnectivityException)
from tf2_geometry_msgs import do_transform_point

TF_ERRORS = (LookupException, ExtrapolationException, ConnectivityException)


class ObstacleAvoider(Node):
    """Obstacle avoidance plus a position-driven geofence, with the robot's
    location and its sensor readings both resolved through TF2.

    Frames involved:
        odom          - fixed reference frame, published by diff_drive_controller
        base_link     - the robot body
        mast          - fixed to base_link
        sensor_mount  - fixed to mast; the LiDAR reports its ranges in this frame

    Nothing here hard-codes a position: the robot's pose comes from the
    odom -> base_link transform, and obstacle detections are converted from
    sensor_mount into odom using the transform between those two frames.
    """

    def __init__(self):
        super().__init__('obstacle_avoider_node')

        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('clear_distance', 0.7)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('cone_angle_deg', 30.0)
        self.declare_parameter('clearing_distance', 2.0)
        self.declare_parameter('direction_scan_deg', 90.0)
        # Which position-driven behaviour runs once the path ahead is clear:
        #   'geofence' - stay within max_distance_from_origin of the origin
        #   'goal'     - drive to (goal_x, goal_y) and stop
        self.declare_parameter('behavior', 'geofence')

        self.declare_parameter('max_distance_from_origin', 5.0)
        self.declare_parameter('boundary_return_ratio', 0.9)

        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.15)

        # Turn in place until within this much of the target bearing, then drive.
        # Lower values behave more like a continuous arc turn (less wheel scrub,
        # more time spent mis-aimed); higher values turn first and drive straight.
        self.declare_parameter('heading_tolerance_deg', 30.0)

        # Start slowing this far out, so the robot can physically stop inside
        # goal_tolerance instead of coasting past it.
        self.declare_parameter('goal_approach_distance', 1.0)

        self.declare_parameter('heading_kp', 1.5)
        self.declare_parameter('reference_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')

        self.enabled = False
        self.state = 'CRUISING'
        self.turn_direction = 1.0
        self.returning = False
        self.goal_reached = False
        self.clear_start_x = 0.0
        self.clear_start_y = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_pose = False

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.obstacle_publisher = self.create_publisher(PointStamped, '/detected_obstacle', 10)
        self.marker_publisher = self.create_publisher(Marker, '/geofence_marker', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_timer = self.create_timer(0.1, self.update_pose_from_tf)
        self.marker_timer = self.create_timer(1.0, self.publish_geofence_marker)

        self.start_service = self.create_service(Trigger, 'start_avoidance', self.start_callback)
        self.stop_service = self.create_service(Trigger, 'stop_avoidance', self.stop_callback)

        self.get_logger().info('Obstacle avoider node ready. Call /start_avoidance to begin.')

    def start_callback(self, request, response):
        self.enabled = True
        self.state = 'CRUISING'
        self.returning = False
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

    def update_pose_from_tf(self):
        """R2: obtain the robot's pose from the odom -> base_link transform."""
        reference_frame = self.get_parameter('reference_frame').value
        robot_frame = self.get_parameter('robot_frame').value

        try:
            transform = self.tf_buffer.lookup_transform(reference_frame, robot_frame, Time())
        except TF_ERRORS as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}', throttle_duration_sec=2.0)
            return

        t = transform.transform.translation
        q = transform.transform.rotation

        self.current_x = t.x
        self.current_y = t.y
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.have_pose = True

        # R3: expose the current position on a topic.
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = reference_frame
        pose_msg.pose.position.x = self.current_x
        pose_msg.pose.position.y = self.current_y
        pose_msg.pose.orientation = q
        self.pose_publisher.publish(pose_msg)

    def report_obstacle_in_reference_frame(self, msg, index):
        """R1: take a LiDAR return, which the sensor reports in its own frame,
        and express it in the fixed reference frame.

        The range/bearing pair is first turned into a point in the scan's frame
        (sensor_mount), then run through the sensor_mount -> odom transform. The
        same obstacle therefore gets a stable world position regardless of where
        the robot was standing when it saw it."""
        reference_frame = self.get_parameter('reference_frame').value
        sensor_frame = msg.header.frame_id
        distance = msg.ranges[index]
        if not math.isfinite(distance):
            return

        bearing = msg.angle_min + index * msg.angle_increment

        point_in_sensor = PointStamped()
        point_in_sensor.header.frame_id = sensor_frame
        point_in_sensor.point.x = distance * math.cos(bearing)
        point_in_sensor.point.y = distance * math.sin(bearing)
        point_in_sensor.point.z = 0.0

        try:
            transform = self.tf_buffer.lookup_transform(reference_frame, sensor_frame, Time())
        except TF_ERRORS as ex:
            self.get_logger().warn(f'Obstacle TF lookup failed: {ex}', throttle_duration_sec=2.0)
            return

        point_in_reference = do_transform_point(point_in_sensor, transform)
        point_in_reference.header.stamp = self.get_clock().now().to_msg()
        self.obstacle_publisher.publish(point_in_reference)

        # Publish the same detection as a frame, so the transform chain
        # sensor_mount -> odom -> detected_obstacle is visible in RViz/view_frames.
        detection = TransformStamped()
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.header.frame_id = reference_frame
        detection.child_frame_id = 'detected_obstacle'
        detection.transform.translation.x = point_in_reference.point.x
        detection.transform.translation.y = point_in_reference.point.y
        detection.transform.translation.z = point_in_reference.point.z
        detection.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(detection)

    def publish_geofence_marker(self):
        """Draw the boundary in the reference frame so R4's behaviour is visible."""
        radius = self.get_parameter('max_distance_from_origin').value
        marker = Marker()
        marker.header.frame_id = self.get_parameter('reference_frame').value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'geofence'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        for i in range(73):
            angle = 2.0 * math.pi * i / 72.0
            marker.points.append(Point(x=radius * math.cos(angle),
                                       y=radius * math.sin(angle), z=0.05))
        self.marker_publisher.publish(marker)

    def choose_turn_direction(self, msg, center_index, scan_deg):
        """Pick the side with more open space, measured over a window wider than
        the detection cone. A dead-centred obstacle ties on the cone minimum
        alone, which would otherwise always default to the same side."""
        half = int(math.radians(scan_deg) / msg.angle_increment)
        n = len(msg.ranges)
        cap = lambda vals: [min(r, msg.range_max) for r in vals if r > 0.0]
        left = cap(msg.ranges[center_index:min(n, center_index + half)])
        right = cap(msg.ranges[max(0, center_index - half):center_index])

        left_min = min(left) if left else msg.range_max
        right_min = min(right) if right else msg.range_max
        if abs(left_min - right_min) > 0.05:
            return 1.0 if right_min < left_min else -1.0

        left_mean = sum(left) / len(left) if left else msg.range_max
        right_mean = sum(right) / len(right) if right else msg.range_max
        return 1.0 if right_mean < left_mean else -1.0

    def scan_callback(self, msg):
        if not self.enabled or not self.have_pose or self.goal_reached:
            return

        behavior = self.get_parameter('behavior').value
        safe_distance = self.get_parameter('safe_distance').value
        clear_distance = self.get_parameter('clear_distance').value
        linear_speed = self.get_parameter('linear_speed').value
        angular_speed = self.get_parameter('angular_speed').value
        cone_angle_deg = self.get_parameter('cone_angle_deg').value
        clearing_distance = self.get_parameter('clearing_distance').value
        direction_scan_deg = self.get_parameter('direction_scan_deg').value
        max_distance = self.get_parameter('max_distance_from_origin').value
        return_ratio = self.get_parameter('boundary_return_ratio').value
        heading_kp = self.get_parameter('heading_kp').value

        num_readings = len(msg.ranges)
        if num_readings == 0:
            return

        cone_angle_rad = math.radians(cone_angle_deg)
        center_index = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        cone_size = int(cone_angle_rad / msg.angle_increment)

        low = max(0, center_index - cone_size)
        high = min(num_readings, center_index + cone_size)
        cone_indices = [i for i in range(low, high) if msg.ranges[i] > 0.0]
        closest_index = min(cone_indices, key=lambda i: msg.ranges[i]) if cone_indices else None
        closest = msg.ranges[closest_index] if closest_index is not None else float('inf')

        if closest_index is not None:
            self.report_obstacle_in_reference_frame(msg, closest_index)

        # R4 state update. This is evaluated on every scan, before the avoidance
        # branches below can return early, so a boundary crossing is noticed as
        # it happens rather than whenever avoidance next finishes.
        if behavior == 'geofence':
            distance_from_origin = math.hypot(self.current_x, self.current_y)
            if self.returning and distance_from_origin < max_distance * return_ratio:
                self.returning = False
                self.get_logger().info(
                    f'Back inside the boundary ({distance_from_origin:.2f} m) — resuming cruise'
                )
            elif not self.returning and distance_from_origin > max_distance:
                self.returning = True
                self.get_logger().info(
                    f'Crossed the {max_distance:.1f} m boundary at '
                    f'({self.current_x:.2f}, {self.current_y:.2f}) — heading back'
                )

        # Arriving is checked before the avoidance states, which return early.
        # Otherwise CLEARING would keep driving its full clearing_distance even
        # after passing the goal, overshooting whenever a goal sits within that
        # distance of an obstacle.
        if behavior == 'goal':
            distance_to_goal = math.hypot(
                self.get_parameter('goal_x').value - self.current_x,
                self.get_parameter('goal_y').value - self.current_y)
            if distance_to_goal < self.get_parameter('goal_tolerance').value:
                self.goal_reached = True
                self.publisher.publish(Twist())
                self.get_logger().info(
                    f'Goal reached at ({self.current_x:.2f}, {self.current_y:.2f})'
                )
                return

        cmd = Twist()

        # Priority 1: obstacle avoidance. Collision safety outranks the geofence,
        # and it runs purely off the LiDAR, so it does not depend on odometry.
        if self.state == 'CRUISING' and closest < safe_distance:
            self.state = 'TURNING'
            self.turn_direction = self.choose_turn_direction(msg, center_index, direction_scan_deg)
            self.get_logger().info(
                f'Obstacle at {closest:.2f} m — turning '
                f'{"left" if self.turn_direction > 0 else "right"}'
            )

        if self.state == 'TURNING':
            if closest > clear_distance:
                self.state = 'CLEARING'
                self.clear_start_x = self.current_x
                self.clear_start_y = self.current_y
                self.get_logger().info(
                    f'Path clear ({closest:.2f} m) — driving {clearing_distance:.2f} m to get past it'
                )
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_direction * angular_speed
                self.publish_cmd(cmd)
                return

        if self.state == 'CLEARING':
            if closest < safe_distance:
                self.state = 'TURNING'
                self.get_logger().info(f'Blocked again at {closest:.2f} m — turning further')
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_direction * angular_speed
                self.publish_cmd(cmd)
                return

            travelled = math.hypot(self.current_x - self.clear_start_x,
                                   self.current_y - self.clear_start_y)
            if travelled < clearing_distance:
                cmd.linear.x = linear_speed
                cmd.angular.z = 0.0
                self.publish_cmd(cmd)
                return

            self.state = 'CRUISING'
            self.get_logger().info('Past the obstacle — resuming cruise')

        # Priority 2: the position-driven behaviour, selected by the 'behavior'
        # parameter. Both steer from the pose obtained via TF2.
        if behavior == 'goal':
            dx = self.get_parameter('goal_x').value - self.current_x
            dy = self.get_parameter('goal_y').value - self.current_y

            heading_error = self.steer_towards(
                cmd, math.atan2(dy, dx), linear_speed, angular_speed, heading_kp)

            self.get_logger().info(
                f'Heading to goal: dist={distance_to_goal:.2f} m, '
                f'heading_err={math.degrees(heading_error):.0f} deg, v={cmd.linear.x:.2f}',
                throttle_duration_sec=2.0
            )
        elif self.returning:
            self.steer_towards(cmd, math.atan2(-self.current_y, -self.current_x),
                               linear_speed, angular_speed, heading_kp)
        else:
            cmd.linear.x = linear_speed
            cmd.angular.z = 0.0

        self.publish_cmd(cmd)

    def publish_cmd(self, cmd):
        """Publish a velocity command, capping forward speed near the goal.

        The cap has to live here rather than in the goal-navigation branch,
        because the avoidance states publish and return before that branch runs.
        CLEARING in particular drives at full speed, so a goal falling inside
        its clearing run was crossed at 1.0 m/s and overshot by the robot's
        0.25 m stopping distance."""
        if cmd.linear.x > 0.0 and self.get_parameter('behavior').value == 'goal':
            distance_to_goal = math.hypot(
                self.get_parameter('goal_x').value - self.current_x,
                self.get_parameter('goal_y').value - self.current_y)
            if distance_to_goal < approach_distance:
                cmd.linear.x *= max(0.15, distance_to_goal / approach_distance)
        self.publish_cmd(cmd)

    def steer_towards(self, cmd, desired_heading, linear_speed, angular_speed, heading_kp):
        """Proportional heading control toward a bearing in the reference frame.

        Beyond heading_tolerance the robot pivots in place rather than arcing,
        so it commits to a heading once and then drives a straight line, instead
        of continuously hunting left and right the whole way there."""
        tolerance = math.radians(self.get_parameter('heading_tolerance_deg').value)

        heading_error = desired_heading - self.current_yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        cmd.angular.z = max(-angular_speed, min(angular_speed, heading_kp * heading_error))

        if abs(heading_error) > tolerance:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = linear_speed
        return heading_error


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
