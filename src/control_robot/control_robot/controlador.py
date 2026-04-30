import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class TrackingControl(Node):
    def __init__(self):
        super().__init__('tracking_control_node')
        
        # --- Parámetros del Control (basados en tu repo) ---
        self.k_v = 0.3  # Ganancia velocidad lineal
        self.k_w = 0.8  # Ganancia velocidad angular
        self.dist_threshold = 0.1 # Tolerancia para llegar al punto (10cm)
        
        # Objetivo (puedes cambiarlo o recibirlo por tópico)
        self.goal_x = 1.0 
        self.goal_y = 1.0

        # Suscripciones y Publicaciones
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.control_loop, 10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Nodo de Tracking Control Iniciado')

    def control_loop(self, msg):
        # 1. Extraer posición actual del odom
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        
        # 2. Extraer orientación actual (Quaternion a Euler/Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        curr_yaw = math.atan2(siny_cosp, cosy_cosp)

        # 3. Cálculo de errores (Lógica de tu repo)
        error_x = self.goal_x - curr_x
        error_y = self.goal_y - curr_y
        distancia = math.sqrt(error_x**2 + error_y**2)

        angle_to_goal = math.atan2(error_y, error_x)
        error_yaw = angle_to_goal - curr_yaw

        # Normalizar ángulo (-pi a pi)
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))

        msg_vel = Twist()

        if distancia > self.dist_threshold:
            # Ley de control proporcional
            v = self.k_v * distancia
            w = self.k_w * error_yaw
            
            # Limitar velocidades por seguridad
            msg_vel.linear.x = min(v, 0.2)  # Max 0.2 m/s
            msg_vel.angular.z = min(max(w, -0.5), 0.5) # Max 0.5 rad/s
        else:
            msg_vel.linear.x = 0.0
            msg_vel.angular.z = 0.0
            self.get_logger().info('¡Objetivo alcanzado!')

        self.pub_vel.publish(msg_vel)

def main(args=None):
    rclpy.init(args=args)
    node = TrackingControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Freno de emergencia al cerrar
        node.pub_vel.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()