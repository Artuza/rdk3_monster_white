import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import csv
import os

class TrackingControlNode(Node):
    def __init__(self):
        super().__init__('tracking_control_node')
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.goal_x = 1.0  # Meta: 1 metro
        self.current_x = 0.0
        self.current_y = 0.0
        self.k_v = 0.5
        self.tolerance = 0.05
        
        # Listas para almacenar datos
        self.history_x = []
        self.history_y = []
        self.history_error = []
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Nodo de Tracking con exportación a RDK listo')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def control_loop(self):
        error = self.goal_x - self.current_x
        
        # Registrar datos en cada ciclo
        self.history_x.append(self.current_x)
        self.history_y.append(self.current_y)
        self.history_error.append(error)
        
        msg = Twist()
        if abs(error) > self.tolerance:
            msg.linear.x = self.k_v * error
            if msg.linear.x > 0.2: msg.linear.x = 0.2
            self.publisher.publish(msg)
        else:
            msg.linear.x = 0.0
            self.publisher.publish(msg)
            self.get_logger().info('Objetivo alcanzado. Exportando datos...')
            self.save_to_csv()
            self.destroy_node()
            rclpy.shutdown()

    def save_to_csv(self):
        # 1. Guardar localmente en la Jetson
        filepath = '/tmp/tracking_results.csv'
        with open(filepath, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'error'])
            for i in range(len(self.history_x)):
                writer.writerow([self.history_x[i], self.history_y[i], self.history_error[i]])
        
        # 2. Enviar a la RDK vía SCP (IP de la RDK .88)
        # Nota: La primera vez podría pedir contraseña de la RDK en la terminal
        os.system(f'scp {filepath} root@192.168.8.88:/root/tracking_results.csv')
        print("Datos enviados exitosamente a la RDK (root@192.168.8.88)")

def main(args=None):
    # Inicializamos ROS 2
    rclpy.init(args=args)
    
    # Creamos el nodo
    node = TrackingControlNode()
    
    try:
        # Mantiene el nodo vivo escuchando tópicos
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Esto ocurre cuando presionas Ctrl+C
        node.get_logger().info('Programa detenido por el usuario (Ctrl+C)')
    except Exception as e:
        # Por si ocurre cualquier otro error
        node.get_logger().error(f'Error inesperado: {e}')
    finally:
        # --- BLOQUE DE GUARDADO ---
        # Estas funciones se ejecutan SIEMPRE al salir
        node.get_logger().info('Guardando datos antes de cerrar...')
        
        try:
            node.save_to_csv()
            node.save_plot()
            node.get_logger().info('¡Datos y gráfica guardados con éxito!')
        except Exception as e:
            node.get_logger().error(f'Error al guardar archivos: {e}')
        
        # Limpieza de ROS
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
