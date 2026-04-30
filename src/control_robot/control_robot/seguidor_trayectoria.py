import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math

class TrackingControl(Node):
    def __init__(self):
        super().__init__('tracking_control_node')
        
        # Perfil de QoS optimizado para el bridge
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Parámetros del control
        self.k_v = 0.3
        self.k_w = 0.8
        self.dist_threshold = 0.1
        self.goal_x = 1.0  
        self.goal_y = 0.0 

        # Suscripción y Publicación
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.control_loop, qos_profile)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        
        self.get_logger().info('--- Nodo de Tracking Iniciado ---')
        self.get_logger().info(f'Objetivo actual: x={self.goal_x}, y={self.goal_y}')

    def control_loop(self, msg):
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        
        # Convertir Quaternion a Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        curr_yaw = math.atan2(siny_cosp, cosy_cosp)

        error_x = self.goal_x - curr_x
        error_y = self.goal_y - curr_y
        distancia = math.sqrt(error_x**2 + error_y**2)

        angle_to_goal = math.atan2(error_y, error_x)
        error_yaw = math.atan2(math.sin(angle_to_goal - curr_yaw), math.cos(angle_to_goal - curr_yaw))

        msg_vel = Twist()

        if distancia > self.dist_threshold:
            # Lógica de movimiento
            msg_vel.linear.x = min(self.k_v * distancia, 0.15) 
            msg_vel.angular.z = min(max(self.k_w * error_yaw, -0.4), 0.4)
            self.pub_vel.publish(msg_vel)
        else:
            # Lógica de llegada y apagado
            msg_vel.linear.x = 0.0
            msg_vel.angular.z = 0.0
            self.pub_vel.publish(msg_vel)
            self.get_logger().info('¡OBJETIVO ALCANZADO! Reiniciando nodo...')
            
            # Matamos el proceso para que el script de bash lo reinicie
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = TrackingControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        # Al presionar Ctrl+C o al llegar al objetivo, enviamos velocidad 0
        stop_msg = Twist()
        node.pub_vel.publish(stop_msg)
        node.get_logger().info('Robot detenido correctamente.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()