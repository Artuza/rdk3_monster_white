import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_CMD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
_LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Built-in trajectory presets (x, y) in metres
_PRESETS: dict[str, list[tuple[float, float]]] = {
    'cuadrado': [
        (1.0, 0.0),
        (1.0, 1.0),
        (0.0, 1.0),
        (0.0, 0.0),
    ],
    'triangulo': [
        (1.0, 0.0),
        (0.5, 0.866),
        (0.0, 0.0),
    ],
    'circulo': [
        (0.5 * math.cos(math.radians(a)), 0.5 * math.sin(math.radians(a)))
        for a in range(0, 360, 22)
    ],
    'ocho': [
        (0.5 * math.cos(t), 0.25 * math.sin(2 * t))
        for t in [math.radians(a) for a in range(0, 360, 20)]
    ],
}


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


class TrajectoryFollower(Node):
    def __init__(self) -> None:
        super().__init__('trajectory_follower')

        # --- Parameters ---
        self.declare_parameter('preset', 'cuadrado')
        self.declare_parameter('loop', False)
        self.declare_parameter('k_v', 0.40)
        self.declare_parameter('k_w', 1.50)
        self.declare_parameter('v_max', 0.20)
        self.declare_parameter('w_max', 0.80)
        self.declare_parameter('dist_tol', 0.10)
        self.declare_parameter('heading_tol', 0.08)
        # Custom waypoints override preset: flat list [x0, y0, x1, y1, ...]
        self.declare_parameter('waypoints', [0.0])

        self._loop        = self.get_parameter('loop').value
        self._k_v         = self.get_parameter('k_v').value
        self._k_w         = self.get_parameter('k_w').value
        self._v_max       = self.get_parameter('v_max').value
        self._w_max       = self.get_parameter('w_max').value
        self._dist_tol    = self.get_parameter('dist_tol').value
        self._heading_tol = self.get_parameter('heading_tol').value
        self._w_min       = 0.05

        # --- State ---
        self._x = 0.0
        self._y = 0.0
        self._th = 0.0
        self._wp_idx = 0
        self._done = False
        self._goal_th = 0.0
        self._waypoints: list[tuple[float, float]] = []  # empty → wait for /goal_pose
        self._log_counter = 0

        # --- Publishers / Subscribers ---
        self.create_subscription(Odometry, '/odom', self._odom_cb, _SENSOR_QOS)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, _CMD_QOS)

        self._pub_vel = self.create_publisher(Twist, '/cmd_vel', _CMD_QOS)
        self._pub_planned = self.create_publisher(Path, '/planned_path', _LATCHED_QOS)
        self._pub_actual = self.create_publisher(Path, '/actual_path', _CMD_QOS)
        self._pub_markers = self.create_publisher(MarkerArray, '/waypoint_markers', _LATCHED_QOS)

        self._actual_path = Path()
        self._actual_path.header.frame_id = 'odom'

        self.create_timer(0.05, self._control_loop)
        self.create_timer(1.0, self._publish_static_viz)

        self.get_logger().info('Trajectory follower ready. Waiting for goal on /goal_pose ...')

    # ------------------------------------------------------------------

    def _goal_cb(self, msg: PoseStamped) -> None:
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._goal_th = math.atan2(siny, cosy)
        self._waypoints = [(x, y)]
        self._wp_idx = 0
        self._done = False
        self._actual_path.poses.clear()
        self.get_logger().warn(
            f'>>> GOAL RECEIVED: x={x:.2f}  y={y:.2f}  θ={math.degrees(self._goal_th):.1f}°  '
            f'robot_at=({self._x:.2f}, {self._y:.2f}) <<<'
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._th = math.atan2(siny, cosy)

    def _publish_static_viz(self) -> None:
        now = self.get_clock().now().to_msg()

        # Planned path line
        path = Path()
        path.header.stamp = now
        path.header.frame_id = 'odom'
        for x, y in self._waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        if self._loop and self._waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = self._waypoints[0][0]
            ps.pose.position.y = self._waypoints[0][1]
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._pub_planned.publish(path)

        # Sphere markers at each waypoint
        markers = MarkerArray()
        for i, (x, y) in enumerate(self._waypoints):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'odom'
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = 0.12
            m.scale.y = 0.12
            m.scale.z = 0.10
            reached = i < self._wp_idx
            m.color.r = 0.0 if reached else 1.0
            m.color.g = 1.0 if reached else 0.5
            m.color.b = 0.0
            m.color.a = 0.9
            markers.markers.append(m)

            # Waypoint index label
            label = Marker()
            label.header = m.header
            label.ns = 'labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.25
            label.pose.orientation.w = 1.0
            label.scale.z = 0.12
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = str(i)
            markers.markers.append(label)

        self._pub_markers.publish(markers)

    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        cmd = Twist()

        if self._done or not self._waypoints:
            self._pub_vel.publish(cmd)
            return

        wp_x, wp_y = self._waypoints[self._wp_idx]
        err_x = wp_x - self._x
        err_y = wp_y - self._y
        dist = math.hypot(err_x, err_y)

        # Waypoint reached
        if dist < self._dist_tol:
            self.get_logger().info(
                f'WP {self._wp_idx} reached  '
                f'target=({wp_x:.2f}, {wp_y:.2f})  '
                f'actual=({self._x:.2f}, {self._y:.2f})  '
                f'err={dist*100:.1f} cm'
            )
            self._wp_idx += 1
            if self._wp_idx >= len(self._waypoints):
                if self._loop:
                    self._wp_idx = 0
                    self.get_logger().info('Loop: returning to first waypoint.')
                else:
                    self._done = True
                    self.get_logger().info('Trajectory complete. Robot stopped.')
            self._pub_vel.publish(cmd)
            return

        # Unicycle controller: always drive forward, steer toward goal
        target_th = math.atan2(err_y, err_x)
        err_th = _wrap(target_th - self._th)

        # Linear speed — reduce when heavily misaligned (facing > 90° away)
        align = math.cos(err_th)  # 1.0 when aligned, 0 at 90°, -1 when opposite
        speed = _clamp(self._k_v * dist * max(align, 0.0), 0.0, self._v_max)
        cmd.linear.x = speed

        # Steering always active
        cmd.angular.z = _clamp(self._k_w * err_th, -self._w_max, self._w_max)

        # Log every 10 cycles (~0.5 s)
        self._log_counter += 1
        if self._log_counter >= 10:
            self._log_counter = 0
            self.get_logger().info(
                f'pos=({self._x:.3f}, {self._y:.3f})  '
                f'goal=({wp_x:.3f}, {wp_y:.3f})  '
                f'dist={dist:.3f} m  '
                f'heading_err={math.degrees(err_th):.1f}°'
            )

        # Append to actual path for RViz
        now_msg = self.get_clock().now().to_msg()
        ps = PoseStamped()
        ps.header.stamp = now_msg
        ps.header.frame_id = 'odom'
        ps.pose.position.x = self._x
        ps.pose.position.y = self._y
        self._actual_path.header.stamp = now_msg
        self._actual_path.poses.append(ps)
        self._pub_actual.publish(self._actual_path)

        self._pub_vel.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub_vel.publish(Twist())
        node.get_logger().info('Node stopped. Robot braked.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
