#!/usr/bin/env python3
"""
DualShock 3 teleop node.
Reads directly from the Linux joystick device (/dev/input/js0).
Publishes geometry_msgs/Twist to /cmd_vel.

DualShock 3 default axis mapping (Linux):
  Axis 0 — Left  stick X  (left=-1, right=+1)
  Axis 1 — Left  stick Y  (up=-1,   down=+1)   ← inverted
  Axis 2 — Right stick X  (left=-1, right=+1)
  Axis 3 — Right stick Y  (up=-1,   down=+1)   ← inverted

Robot mapping (ROS convention: x=forward, y=left, z=up):
  vx  ← -axis[1]  (negate: stick up → forward)
  vy  ← -axis[0]  (negate: stick left → positive vy)
  wz  ← -axis[2]  (negate: stick left → counterclockwise)
"""

import struct
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Linux joystick event types
_JS_BUTTON = 0x01
_JS_AXIS   = 0x02
_JS_INIT   = 0x80
_AXIS_MAX  = 32767.0
_EVENT_FMT = 'IhBB'   # time(uint32), value(int16), type(uint8), number(uint8)
_EVENT_SZ  = struct.calcsize(_EVENT_FMT)


class DS3TeleopNode(Node):

    def __init__(self):
        super().__init__('ds3_teleop_node')

        self.declare_parameter('device',       '/dev/input/js0')
        self.declare_parameter('publish_rate', 20.0)    # Hz
        self.declare_parameter('max_vx',       0.5)     # m/s
        self.declare_parameter('max_vy',       0.5)     # m/s
        self.declare_parameter('max_wz',       1.5)     # rad/s
        self.declare_parameter('deadzone',     0.08)
        # Axis indices
        self.declare_parameter('axis_vx',  1)   # left stick Y
        self.declare_parameter('axis_vy',  0)   # left stick X
        self.declare_parameter('axis_wz',  2)   # right stick X
        # Scale signs  (+1 or -1) to correct direction without changing index
        self.declare_parameter('sign_vx', -1.0)
        self.declare_parameter('sign_vy', -1.0)
        self.declare_parameter('sign_wz', -1.0)

        self._device  = self.get_parameter('device').value
        self._max_vx  = self.get_parameter('max_vx').value
        self._max_vy  = self.get_parameter('max_vy').value
        self._max_wz  = self.get_parameter('max_wz').value
        self._dz      = self.get_parameter('deadzone').value
        self._ax_vx   = self.get_parameter('axis_vx').value
        self._ax_vy   = self.get_parameter('axis_vy').value
        self._ax_wz   = self.get_parameter('axis_wz').value
        self._sg_vx   = self.get_parameter('sign_vx').value
        self._sg_vy   = self.get_parameter('sign_vy').value
        self._sg_wz   = self.get_parameter('sign_wz').value
        rate          = self.get_parameter('publish_rate').value

        # Shared axis state
        self._axes    : dict[int, float] = {}
        self._lock    = threading.Lock()
        self._js_ok   = False

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        _qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(Twist, '/cmd_vel', _qos)
        self.create_timer(1.0 / rate, self._publish_cb)

        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        self.get_logger().info(
            f'DS3 teleop ready | device={self._device} | '
            f'max vx={self._max_vx} vy={self._max_vy} wz={self._max_wz}')

    # ── Joystick reader (background thread) ──────────────────────────────────

    def _read_loop(self):
        while rclpy.ok():
            try:
                with open(self._device, 'rb') as f:
                    self._js_ok = True
                    self.get_logger().info('Joystick connected.')
                    while rclpy.ok():
                        raw = f.read(_EVENT_SZ)
                        if len(raw) < _EVENT_SZ:
                            break
                        _, value, etype, number = struct.unpack(_EVENT_FMT, raw)
                        if (etype & ~_JS_INIT) == _JS_AXIS:
                            with self._lock:
                                self._axes[number] = value / _AXIS_MAX
            except OSError as e:
                if self._js_ok:
                    self.get_logger().warn(f'Joystick disconnected: {e}')
                    self._js_ok = False
                    with self._lock:
                        self._axes.clear()
                import time; time.sleep(1.0)   # retry every second

    # ── Publish timer ─────────────────────────────────────────────────────────

    def _publish_cb(self):
        with self._lock:
            raw_vx = self._axes.get(self._ax_vx, 0.0)
            raw_vy = self._axes.get(self._ax_vy, 0.0)
            raw_wz = self._axes.get(self._ax_wz, 0.0)

        cmd = Twist()
        cmd.linear.x  = self._sg_vx * self._scale(raw_vx) * self._max_vx
        cmd.linear.y  = self._sg_vy * self._scale(raw_vy) * self._max_vy
        cmd.angular.z = self._sg_wz * self._scale(raw_wz) * self._max_wz
        self._pub.publish(cmd)

    # ── Deadzone + rescale ────────────────────────────────────────────────────

    def _scale(self, value: float) -> float:
        """Apply deadzone and rescale so output spans [0, 1] outside deadzone."""
        if abs(value) < self._dz:
            return 0.0
        sign = 1.0 if value > 0.0 else -1.0
        return sign * (abs(value) - self._dz) / (1.0 - self._dz)


def main(args=None):
    rclpy.init(args=args)
    node = DS3TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero before shutting down
        stop = Twist()
        node._pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()