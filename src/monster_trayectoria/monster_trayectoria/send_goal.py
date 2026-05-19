"""
Usage:
    ros2 run monster_trayectoria send_goal <x> <y>

Example:
    ros2 run monster_trayectoria send_goal 2.0 1.5
"""
import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped


_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


def main(args=None) -> None:
    if len(sys.argv) < 3:
        print('Usage: ros2 run monster_trayectoria send_goal <x> <y>')
        sys.exit(1)

    x = float(sys.argv[1])
    y = float(sys.argv[2])

    rclpy.init(args=args)
    node = Node('goal_sender')
    pub = node.create_publisher(PoseStamped, '/goal_pose', _QOS)

    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'odom'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.0
    msg.pose.orientation.w = 1.0

    # Wait one cycle so the publisher is connected before sending
    import time
    time.sleep(0.5)
    pub.publish(msg)
    node.get_logger().info(f'Goal sent → x={x}  y={y}')

    node.destroy_node()
    rclpy.shutdown()
