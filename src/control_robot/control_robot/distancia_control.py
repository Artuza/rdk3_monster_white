import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy
)

import math
import csv
import os


class ControlDistanciaPreciso(Node):

    def __init__(self):

        super().__init__('control_distancia_node')

        # ==========================================
        # INPUT DE DISTANCIA
        # ==========================================
        try:

            dist_input = input(
                "\n>>> Introduce la distancia objetivo en metros (ej. 0.5): "
            )

            self.target_dist = float(dist_input)

        except ValueError:

            print("Entrada inválida. Usando 0.5m")

            self.target_dist = 0.5

        # ==========================================
        # QoS
        # ==========================================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=50
        )

        # ==========================================
        # SUBSCRIBER ODOM
        # ==========================================
        self.sub = self.create_subscription(
            Odometry,
            '/odom_raw',
            self.odom_callback,
            qos
        )

        # ==========================================
        # PUBLISHER
        # IMPORTANTE:
        # base_node escucha vel_raw
        # ==========================================
        self.pub = self.create_publisher(
            Twist,
            '/vel_raw',
            10
        )

        # ==========================================
        # VARIABLES DE POSICIÓN
        # ==========================================
        self.start_x = None
        self.start_y = None

        self.current_x = 0.0
        self.current_y = 0.0

        # ==========================================
        # VARIABLES DE ORIENTACIÓN
        # ==========================================
        self.start_yaw = None
        self.current_yaw = 0.0

        # ==========================================
        # CONTROL
        # ==========================================
        self.k_theta = 0.4
        self.max_angular = 0.3

        self.linear_speed = 0.10

        self.moving = True

        # ==========================================
        # HISTORIAL
        # ==========================================
        self.history = []

        # ==========================================
        # TIMER
        # ==========================================
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            f'Objetivo configurado: {self.target_dist:.2f} m'
        )

        self.get_logger().info(
            'Esperando odometría...'
        )

    # ==================================================
    # QUATERNION -> YAW
    # ==================================================
    def get_yaw_from_quaternion(self, q):

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # ==================================================
    # CALLBACK ODOM
    # ==================================================
    def odom_callback(self, msg):

        # Posición
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Orientación
        q = msg.pose.pose.orientation

        self.current_yaw = (
            self.get_yaw_from_quaternion(q)
        )

        # Guardar referencias iniciales
        if self.start_x is None:

            self.start_x = self.current_x
            self.start_y = self.current_y

            self.start_yaw = self.current_yaw

            self.get_logger().info(
                'Punto inicial fijado'
            )

            self.get_logger().info(
                f'X0={self.start_x:.3f} '
                f'Y0={self.start_y:.3f} '
                f'Yaw0={self.start_yaw:.3f}'
            )

    # ==================================================
    # LOOP DE CONTROL
    # ==================================================
    def control_loop(self):

        if (
            self.start_x is None or
            self.start_y is None or
            self.start_yaw is None or
            not self.moving
        ):
            return

        # ==========================================
        # DISTANCIA EUCLIDIANA
        # ==========================================
        distancia_recorrida = math.sqrt(
            (self.current_x - self.start_x) ** 2 +
            (self.current_y - self.start_y) ** 2
        )

        # ==========================================
        # ERROR LINEAL
        # ==========================================
        error_lineal = (
            self.target_dist -
            distancia_recorrida
        )

        # ==========================================
        # ERROR ANGULAR
        # ==========================================
        error_yaw = (
            self.start_yaw -
            self.current_yaw
        )

        # Normalización [-pi, pi]
        error_yaw = math.atan2(
            math.sin(error_yaw),
            math.cos(error_yaw)
        )

        # ==========================================
        # MONITOR
        # ==========================================
        self.get_logger().info(
            f'Progreso: '
            f'{distancia_recorrida:.3f}m '
            f'/ {self.target_dist:.3f}m | '
            f'Error: {error_lineal:.3f}m | '
            f'Yaw Error: {error_yaw:.3f}',
            throttle_duration_sec=0.5
        )

        # ==========================================
        # GUARDAR HISTORIAL
        # ==========================================
        self.history.append([
            distancia_recorrida,
            error_lineal,
            self.current_x,
            self.current_y,
            self.current_yaw,
            error_yaw
        ])

        # ==========================================
        # CONTROL
        # ==========================================
        msg = Twist()

        # Tolerancia
        if error_lineal > 0.02:

            # Velocidad lineal
            msg.linear.x = self.linear_speed

            # Corrección angular
            msg.angular.z = (
                self.k_theta *
                error_yaw
            )

            # Saturación angular
            if msg.angular.z > self.max_angular:
                msg.angular.z = self.max_angular

            elif msg.angular.z < -self.max_angular:
                msg.angular.z = -self.max_angular

            self.pub.publish(msg)

        else:

            self.get_logger().info(
                f'META ALCANZADA: '
                f'{distancia_recorrida:.3f}m'
            )

            self.moving = False

            self.stop_robot()

            self.guardar_datos()

    # ==================================================
    # STOP ROBOT
    # ==================================================
    def stop_robot(self):

        msg = Twist()

        msg.linear.x = 0.0
        msg.angular.z = 0.0

        for _ in range(10):

            self.pub.publish(msg)

    # ==================================================
    # GUARDAR CSV
    # ==================================================
    def guardar_datos(self):

        ruta_local = (
            '/home/jetson/datos_control.csv'
        )

        try:

            with open(
                ruta_local,
                'w',
                newline=''
            ) as f:

                writer = csv.writer(f)

                writer.writerow([
                    'distancia',
                    'error_lineal',
                    'x',
                    'y',
                    'yaw',
                    'error_yaw'
                ])

                writer.writerows(
                    self.history
                )

            self.get_logger().info(
                f'CSV guardado en: {ruta_local}'
            )

            # SCP opcional
            os.system(
                f'scp {ruta_local} '
                f'root@10.0.0.1:/root/muestras_caracterizacion.csv &'
            )

        except Exception as e:

            self.get_logger().error(
                f'Error guardando CSV: {e}'
            )


# ======================================================
# MAIN
# ======================================================
def main(args=None):

    rclpy.init(args=args)

    node = ControlDistanciaPreciso()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            'Detenido manualmente'
        )

        node.stop_robot()

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()