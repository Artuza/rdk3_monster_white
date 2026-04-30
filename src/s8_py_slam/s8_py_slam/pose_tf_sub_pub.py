import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from s7_robot_network_interface.msg import AMRStatus
import tf_transformations
import math

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_tf_sub_pub', namespace='rdk_x3_amr')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Publishers ---
        # Opt 1: Publisher for a PoseStamped message
        self.pose_pub = self.create_publisher(PoseStamped, '/rdk_x3_amr/pose_stamped', 10)
        # Opt 2: Publisher for a custom AMRStatus message
        self.robot_pub = self.create_publisher(AMRStatus, '/rdk_x3_amr/robot_status', 10)

        # --- Timer for periodic updates (~15 Hz) ---
        self.timer = self.create_timer(0.066, self.publish_pose)

    def publish_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', now)

            # Extract position and orientation from the transform
            position = Point()
            position.x = trans.transform.translation.x
            position.y = trans.transform.translation.y
            position.z = trans.transform.translation.z
            quaternion = Quaternion()
            quaternion.x = trans.transform.rotation.x
            quaternion.y = trans.transform.rotation.y
            quaternion.z = trans.transform.rotation.z
            quaternion.w = trans.transform.rotation.w

            # Opt 1: Publish PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position = position # Or simply trans.transform.translation
            pose_msg.pose.orientation = quaternion # Or simply trans.transform.rotation
            self.pose_pub.publish(pose_msg)

            # Opt 2: Publish AMRStatus message
            status_msg = AMRStatus()
            status_msg.timestamp = self.get_clock().now().to_msg()
            status_msg.robot_id = 1 # Default ID for the AMR
            status_msg.task_stage = 9 # Exploring
            status_msg.pose.x = position.x # Or simply trans.transform.translation.x
            status_msg.pose.y = position.y # Or simply trans.transform.translation.y
            qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w
            # Get roll, pitch, yaw in degrees
            euler = tf_transformations.euler_from_quaternion((qx, qy, qz, qw))
            status_msg.pose.theta = euler[2]
            # status_msg.align_yaw = True
            self.robot_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

def main():
    rclpy.init()
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Exception caught: {e}')
    finally:        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
