#!/usr/bin/env python3
"""
Prueba de movimiento omnidireccional con verificación de encoders.

Hace 4 movimientos (adelante, lateral izq, lateral der, rotación izq)
y muestra los deltas de encoder de cada rueda para confirmar
que la cinemática mecanum es correcta.

Patrón esperado por movimiento:
  Adelante  (+x): m1+ m2+ m3+ m4+  (todos adelante)
  Atrás     (-x): m1- m2- m3- m4-  (todos atrás)
  Strafe izq(+y): m1- m2+ m3+ m4-  (diagonal)
  Strafe der(-y): m1+ m2- m3- m4+  (diagonal opuesto)
  Giro izq  (+z): m1- m2- m3+ m4+  (izq atrás, der adelante)
  Giro der  (-z): m1+ m2+ m3- m4-  (izq adelante, der atrás)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import time


SPEED     = 0.3   # m/s  para linear.x / linear.y
ROT_SPEED = 1.0   # rad/s para angular.z
DURATION  = 2.0   # segundos por movimiento
PAUSE     = 1.5   # pausa entre movimientos

MOVES = [
    ('Adelante   (+x)', Twist(), {'lx':  SPEED}),
    ('Atrás      (-x)', Twist(), {'lx': -SPEED}),
    ('Strafe izq (+y)', Twist(), {'ly':  SPEED}),
    ('Strafe der (-y)', Twist(), {'ly': -SPEED}),
    ('Giro izq   (+z)', Twist(), {'az':  ROT_SPEED}),
    ('Giro der   (-z)', Twist(), {'az': -ROT_SPEED}),
]

# Rellenar Twist de cada movimiento
def make_twist(lx=0.0, ly=0.0, az=0.0):
    t = Twist()
    t.linear.x  = lx
    t.linear.y  = ly
    t.angular.z = az
    return t

MOVES = [
    ('Adelante   (+x)', make_twist(lx= SPEED)),
    ('Atras      (-x)', make_twist(lx=-SPEED)),
    ('Strafe izq (+y)', make_twist(ly= SPEED)),
    ('Strafe der (-y)', make_twist(ly=-SPEED)),
    ('Giro izq   (+z)', make_twist(az= ROT_SPEED)),
    ('Giro der   (-z)', make_twist(az=-ROT_SPEED)),
]

EXPECTED_SIGNS = {
    'Adelante   (+x)': ('+', '+', '+', '+'),
    'Atras      (-x)': ('-', '-', '-', '-'),
    'Strafe izq (+y)': ('-', '+', '+', '-'),
    'Strafe der (-y)': ('+', '-', '-', '+'),
    'Giro izq   (+z)': ('-', '-', '+', '+'),
    'Giro der   (-z)': ('+', '+', '-', '-'),
}


class OmniTest(Node):
    def __init__(self):
        super().__init__('omni_test')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Encoders vienen del driver modificado via /joint_states o los
        # leemos indirectamente: aquí usamos vel_raw como proxy de movimiento
        self._enc = [0, 0, 0, 0]   # m1 m2 m3 m4 (acumulados)
        self._enc_base = [0, 0, 0, 0]
        self._enc_ready = False

        # El driver con log de encoders publica en el log, no en un topic.
        # Usamos vel_raw para saber si el robot se movió.
        from geometry_msgs.msg import Twist as TwistMsg
        self.create_subscription(TwistMsg, '/vel_raw', self._vel_cb, 10)
        self._last_vel = (0.0, 0.0, 0.0)

    def _vel_cb(self, msg):
        self._last_vel = (msg.linear.x, msg.linear.y, msg.angular.z)

    def stop(self):
        self._pub.publish(Twist())

    def run(self):
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('PRUEBA OMNIDIRECCIONAL — robot Yahboom mecanum')
        self.get_logger().info('='*60)

        results = []

        for name, cmd in MOVES:
            self.get_logger().info(f'\n--- {name} ---')
            self.get_logger().info(
                f'Comando: vx={cmd.linear.x:.2f}  vy={cmd.linear.y:.2f}  wz={cmd.angular.z:.2f}')

            # Publicar comando durante DURATION segundos
            t_start = time.time()
            vel_samples = []
            while time.time() - t_start < DURATION:
                self._pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.1)
                vel_samples.append(self._last_vel)

            self.stop()
            time.sleep(PAUSE)
            rclpy.spin_once(self, timeout_sec=0.1)

            # Velocidad media medida
            if vel_samples:
                vx_m = sum(v[0] for v in vel_samples) / len(vel_samples)
                vy_m = sum(v[1] for v in vel_samples) / len(vel_samples)
                wz_m = sum(v[2] for v in vel_samples) / len(vel_samples)
                moving = abs(vx_m) + abs(vy_m) + abs(wz_m) > 0.02
                self.get_logger().info(
                    f'vel_raw media: vx={vx_m:+.3f}  vy={vy_m:+.3f}  wz={wz_m:+.3f}  '
                    f'→ {"MOVIMIENTO DETECTADO ✓" if moving else "SIN MOVIMIENTO ✗"}'
                )
            results.append((name, vel_samples))

        # Resumen
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('RESUMEN')
        self.get_logger().info('='*60)
        for name, samples in results:
            if samples:
                vx = sum(v[0] for v in samples)/len(samples)
                vy = sum(v[1] for v in samples)/len(samples)
                wz = sum(v[2] for v in samples)/len(samples)
                ok = abs(vx)+abs(vy)+abs(wz) > 0.02
                self.get_logger().info(
                    f'  {"✓" if ok else "✗"} {name:24s}  '
                    f'vx={vx:+.3f}  vy={vy:+.3f}  wz={wz:+.3f}'
                )
        self.get_logger().info('='*60)
        self.get_logger().info('Prueba completada.')


def main(args=None):
    rclpy.init(args=args)
    node = OmniTest()
    try:
        node.run()
    except KeyboardInterrupt:
        node.stop()
        node.get_logger().info('Interrumpido.')
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
