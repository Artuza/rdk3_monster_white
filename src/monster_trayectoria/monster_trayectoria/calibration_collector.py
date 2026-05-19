"""
Recolector de muestras de calibración para el robot Yahboom mecanum.

Mueve el robot 1 m hacia adelante (o 90° de giro), para, pide al usuario
que ingrese la medición real, luego vuelve a la posición original y repite.

Al finalizar calcula:
  - Factor de corrección (bias)      : mean(real / comandado)
  - Ruido de proceso Q               : var(real - comandado)
  - Guarda resultados en calibration.yaml

Uso:
  ros2 run monster_trayectoria calibration_collector
  ros2 run monster_trayectoria calibration_collector --ros-args -p mode:=angle
"""

import csv
import math
import os
import threading
import time
from datetime import datetime

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node

# ── Parámetros por defecto ────────────────────────────────────────────────────
N_SAMPLES   = 50
DIST_M      = 1.0    # metros a comandar por muestra
ANGLE_DEG   = 90.0   # grados a comandar por muestra
V_MAX       = 0.2    # m/s (despacio para precisión)
W_MAX       = 1.0    # rad/s
STOP_TIME   = 2.0    # segundos de pausa entre movimientos
SPIN_FACTOR = 1.4    # igual que drift_square_robot


class CalibrationCollector(Node):
    def __init__(self) -> None:
        super().__init__('calibration_collector')

        self.declare_parameter('mode',      'distance')   # 'distance' | 'angle'
        self.declare_parameter('n_samples', N_SAMPLES)
        self.declare_parameter('dist_m',    DIST_M)
        self.declare_parameter('angle_deg', ANGLE_DEG)
        self.declare_parameter('v_max',     V_MAX)
        self.declare_parameter('w_max',     W_MAX)

        self._mode      = self.get_parameter('mode').value
        self._n         = int(self.get_parameter('n_samples').value)
        self._dist      = float(self.get_parameter('dist_m').value)
        self._angle_deg = float(self.get_parameter('angle_deg').value)
        self._v         = float(self.get_parameter('v_max').value)
        self._w         = float(self.get_parameter('w_max').value)

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._running = True

        # Tiempos de movimiento
        self._drive_time = self._dist / self._v
        self._spin_time  = math.radians(self._angle_deg) / self._w * SPIN_FACTOR

        self.get_logger().info(
            f'Calibración  modo={self._mode}  n={self._n}  '
            f'{"dist=" + str(self._dist) + "m" if self._mode == "distance" else "angle=" + str(self._angle_deg) + "°"}'
        )

    # ── Publicar velocidad durante `duration` segundos ────────────────────────
    def _publish_for(self, vx: float = 0.0, wz: float = 0.0, duration: float = 1.0) -> None:
        cmd = Twist()
        cmd.linear.x  = vx
        cmd.angular.z = wz
        end = time.time() + duration
        while time.time() < end and self._running:
            self._pub.publish(cmd)
            time.sleep(0.05)
        self._pub.publish(Twist())   # stop

    def _pause(self, t: float = STOP_TIME) -> None:
        self._publish_for(0.0, 0.0, t)
        time.sleep(0.2)

    # ── Ciclo de recolección (corre en hilo aparte) ───────────────────────────
    def collect(self) -> None:
        samples_cmd, samples_real = [], []

        time.sleep(1.0)   # espera a que el nodo esté listo

        print('\n' + '=' * 60)
        if self._mode == 'distance':
            print(f'CALIBRACIÓN DE DISTANCIA — {self._n} muestras')
            print(f'El robot avanzará {self._dist} m, para, tú mides y escribes')
            print(f'el valor real. Luego vuelve solo. Pon una cinta métrica.')
        else:
            print(f'CALIBRACIÓN DE ÁNGULO — {self._n} muestras')
            print(f'El robot girará {self._angle_deg}°, para, tú mides con')
            print(f'transportador o app de brújula y escribes el valor real.')
        print('=' * 60 + '\n')
        input('Presiona ENTER cuando el robot esté listo...')

        for i in range(self._n):
            print(f'\n--- Muestra {i + 1}/{self._n} ---')

            if self._mode == 'distance':
                print(f'  Avanzando {self._dist} m...')
                self._publish_for(self._v, 0.0, self._drive_time)
                self._pause(STOP_TIME)

                raw = input(f'  ¿Distancia real recorrida (metros)? ')
                try:
                    real = float(raw.replace(',', '.'))
                except ValueError:
                    print('  Valor inválido, muestra omitida')
                    self._publish_for(-self._v, 0.0, self._drive_time)
                    self._pause(STOP_TIME)
                    continue

                samples_cmd.append(self._dist)
                samples_real.append(real)
                print(f'  Guardado: comandado={self._dist:.2f}m  real={real:.3f}m  error={real - self._dist:+.3f}m')

                print('  Volviendo a la posición inicial...')
                self._publish_for(-self._v, 0.0, self._drive_time)
                self._pause(STOP_TIME)

            else:  # angle
                print(f'  Girando {self._angle_deg}° a la izquierda...')
                self._publish_for(0.0, self._w, self._spin_time)
                self._pause(STOP_TIME)

                raw = input(f'  ¿Ángulo real girado (grados)? ')
                try:
                    real = float(raw.replace(',', '.'))
                except ValueError:
                    print('  Valor inválido, muestra omitida')
                    self._publish_for(0.0, -self._w, self._spin_time)
                    self._pause(STOP_TIME)
                    continue

                samples_cmd.append(self._angle_deg)
                samples_real.append(real)
                print(f'  Guardado: comandado={self._angle_deg:.1f}°  real={real:.1f}°  error={real - self._angle_deg:+.1f}°')

                print('  Volviendo a la orientación inicial...')
                self._publish_for(0.0, -self._w, self._spin_time)
                self._pause(STOP_TIME)

        self._running = False
        self._save(samples_cmd, samples_real)

    # ── Guardar CSV y YAML ────────────────────────────────────────────────────
    def _save(self, commanded: list, real: list) -> None:
        if not commanded:
            self.get_logger().warn('Sin muestras para guardar')
            return

        n      = len(commanded)
        errors = [r - c for c, r in zip(commanded, real)]
        ratios = [r / c for c, r in zip(commanded, real) if c != 0]

        mean_err   = sum(errors) / n
        var_err    = sum((e - mean_err) ** 2 for e in errors) / max(n - 1, 1)
        std_err    = math.sqrt(var_err)
        mean_ratio = sum(ratios) / len(ratios)

        outdir = os.path.expanduser('~/ros2_ws/logs')
        os.makedirs(outdir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # CSV con las muestras
        csv_path = os.path.join(outdir, f'calib_{self._mode}_{stamp}.csv')
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['sample', 'commanded', 'real', 'error', 'ratio'])
            for i, (c, r) in enumerate(zip(commanded, real)):
                w.writerow([i + 1, f'{c:.4f}', f'{r:.4f}', f'{r - c:.4f}', f'{r / c:.4f}'])

        # YAML con parámetros del Kalman
        yaml_path = os.path.expanduser('~/ros2_ws/calibration.yaml')
        # Cargar valores existentes si los hay
        existing = {}
        if os.path.exists(yaml_path):
            with open(yaml_path) as f:
                existing = yaml.safe_load(f) or {}

        existing[self._mode] = {
            'n_samples':        n,
            'mean_error':       round(mean_err, 5),
            'std_error':        round(std_err, 5),
            'variance':         round(var_err, 6),
            'correction_factor': round(mean_ratio, 5),
            'q_noise':          round(var_err, 6),    # ruido de proceso → Q
        }

        with open(yaml_path, 'w') as f:
            yaml.dump(existing, f, default_flow_style=False)

        print('\n' + '=' * 60)
        print(f'RESULTADOS — {self._mode.upper()} ({n} muestras)')
        print(f'  Error medio   : {mean_err:+.4f}')
        print(f'  Desv. estándar: {std_err:.4f}')
        print(f'  Factor correc.: {mean_ratio:.4f}  (real/comandado)')
        print(f'  Q (proc.noise): {var_err:.6f}')
        print(f'\n  CSV  → {csv_path}')
        print(f'  YAML → {yaml_path}')
        print('=' * 60)
        self.get_logger().info('Calibración completa')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibrationCollector()

    # Hilo de ROS2 spin (para que el publisher funcione)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.collect()
    except KeyboardInterrupt:
        node.get_logger().info('Calibración cancelada')
    finally:
        node._pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
