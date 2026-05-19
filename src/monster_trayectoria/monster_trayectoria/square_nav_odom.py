"""
Navegador cuadrado 4-waypoints — feedback: Odometría (/odom)

Misma lógica de control que square_nav_optitrack.py, distinta fuente de posición.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from monster_trayectoria.square_nav_base import SquareNavigatorBase

_ODOM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class SquareNavOdom(SquareNavigatorBase):
    def __init__(self):
        super().__init__('square_nav_odom')
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, _ODOM_QOS
        )
        self.get_logger().info(
            'Fuente de posición: Odometría (/odom)\n'
            + self._startup_banner()
        )

    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._update_pose(x, y, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = SquareNavOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
