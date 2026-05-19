"""
Navegador cuadrado 4-waypoints — feedback: OptiTrack (/optitrack/rigid_body)

Misma lógica de control que square_nav_odom.py, distinta fuente de posición.
"""

import csv, json, math, os
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from monster_trayectoria.square_nav_base import SquareNavigatorBase

_OPTI_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class SquareNavOptitrack(SquareNavigatorBase):
    def __init__(self):
        super().__init__('square_nav_optitrack')
        self.create_subscription(
            PoseStamped, '/optitrack/rigid_body', self._pose_cb, _OPTI_QOS
        )
        self.get_logger().info(
            'Fuente de posición: OptiTrack (/optitrack/rigid_body)\n'
            + self._startup_banner()
        )

    def _pose_cb(self, msg: PoseStamped) -> None:
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._update_pose(x, y, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = SquareNavOptitrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
