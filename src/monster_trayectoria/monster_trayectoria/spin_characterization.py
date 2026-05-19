#!/usr/bin/env python3
"""
Caracterización de giros: compara ángulo real (OptiTrack) vs predicción del KF
para calibrar angle_factor y entender la dinámica de rotación del robot.

Secuencia por repetición:
  WAIT  → esperar OptiTrack válido
  SPIN  → girar a w_cmd durante t_spin segundos
  COAST → cmd=0, medir inercia hasta t_coast segundos
  PAUSE → esperar t_pause antes del siguiente giro
  DONE  → imprimir resumen y sugerir calibración

Salida: CSV + resumen en consola + spin_calibration_suggestion.yaml
"""

import csv
import math
import os
import statistics
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy,
                        QoSProfile, ReliabilityPolicy)


# ── KalmanPose mínimo (predict-only durante SPIN para ver raw dead-reckoning) ──
class KalmanPose:
    def __init__(self):
        self.x = np.zeros(3)
        self.P = np.eye(3) * 0.1
        self.Q = np.diag([0.01, 0.01, 0.01])
        self.angle_factor = 1.0   # se carga de calibration.yaml si existe

    def predict(self, wz: float, dt: float) -> None:
        wz_c = wz * self.angle_factor
        self.x[2] += wz_c * dt
        self.x[2]  = math.atan2(math.sin(self.x[2]), math.cos(self.x[2]))
        self.P[2, 2] += self.Q[2, 2]

    @property
    def yaw(self) -> float:
        return float(self.x[2])


_CMD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=10)

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST, depth=10)


class SpinCharacterizer(Node):

    def __init__(self) -> None:
        super().__init__('spin_characterizer')

        self.declare_parameter('w_cmd',      1.0)   # rad/s velocidad de giro
        self.declare_parameter('t_spin',     2.5)   # s girando (debe cubrir >90°)
        self.declare_parameter('t_coast',    2.0)   # s midiendo inercia tras parar
        self.declare_parameter('t_pause',    3.0)   # s entre repeticiones
        self.declare_parameter('n_reps',     6)     # número de giros
        self.declare_parameter('rigid_name', '')

        self._w_cmd      = float(self.get_parameter('w_cmd').value)
        self._t_spin     = float(self.get_parameter('t_spin').value)
        self._t_coast    = float(self.get_parameter('t_coast').value)
        self._t_pause    = float(self.get_parameter('t_pause').value)
        self._n_reps     = int(self.get_parameter('n_reps').value)
        self._rigid_name = self.get_parameter('rigid_name').value.strip()

        self._pub_vel = self.create_publisher(Twist, '/cmd_vel', _CMD_QOS)
        self.create_subscription(PoseStamped, '/optitrack/rigid_body',
                                 self._cb_optitrack, _SENSOR_QOS)
        self.create_subscription(Imu, '/imu/data_raw', self._cb_imu, 10)

        # ── OptiTrack ──────────────────────────────────────────────────────────
        self._opt_valid      = False
        self._opt_origin_yaw = 0.0
        self._opt_yaw        = 0.0   # yaw relativo al origen del programa
        self._opt_spin_start = 0.0   # opt_yaw al inicio de cada SPIN

        # ── IMU ───────────────────────────────────────────────────────────────
        self._imu_yaw    = 0.0
        self._imu_last_t = -1.0

        # ── KF (predict-only, sin updates de OptiTrack) ───────────────────────
        self._kf = KalmanPose()
        calib = os.path.expanduser('~/ros2_ws/calibration.yaml')
        if os.path.exists(calib):
            try:
                import yaml
                with open(calib) as f:
                    d = yaml.safe_load(f) or {}
                if 'angle' in d:
                    self._kf.angle_factor = d['angle'].get('correction_factor', 1.0)
                    self.get_logger().info(
                        f'KF angle_factor={self._kf.angle_factor:.4f} (desde calibration.yaml)')
            except Exception:
                pass
        self._kf_last_t       = -1.0
        self._kf_spin_start   = 0.0
        self._prev_cmd_wz     = 0.0

        # ── Detección de spin-up ───────────────────────────────────────────────
        self._spin_start_t    = 0.0   # tiempo absoluto de inicio del SPIN
        self._spinup_detected = False
        self._spinup_delay_s  = 0.0   # s hasta que opt_dyaw supera 2°

        # ── Estado ────────────────────────────────────────────────────────────
        self._state       = 'WAIT'
        self._rep         = 0
        self._phase_start = self.get_clock().now()
        self._t0          = self.get_clock().now()

        # ── Resultados ────────────────────────────────────────────────────────
        self._results: list[dict] = []

        # ── CSV ───────────────────────────────────────────────────────────────
        stamp    = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir  = os.path.expanduser('~/ros2_ws/logs')
        os.makedirs(log_dir, exist_ok=True)
        self._log_path = os.path.join(log_dir, f'spin_char_{stamp}.csv')
        self._csv_file = open(self._log_path, 'w', newline='')
        self._csv_w    = csv.writer(self._csv_file)
        self._csv_w.writerow([
            't_s', 'rep', 'state', 'elapsed_s',
            'cmd_wz',
            'opt_dyaw_deg', 'imu_yaw_deg', 'kf_dyaw_deg',
            'kf_err_deg',   # kf_dyaw - opt_dyaw
            'event',
        ])

        self.create_timer(0.05, self._loop)
        self.get_logger().info(
            f'SpinCharacterizer  w_cmd={self._w_cmd:.2f} rad/s  '
            f't_spin={self._t_spin:.1f}s  n_reps={self._n_reps}  '
            f'CSV={self._log_path}')
        self.get_logger().info('Esperando OptiTrack...')

    # ── Tiempo ────────────────────────────────────────────────────────────────

    def _now_s(self) -> float:
        return (self.get_clock().now() - self._t0).nanoseconds * 1e-9

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._phase_start).nanoseconds * 1e-9

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_optitrack(self, msg: PoseStamped) -> None:
        if self._rigid_name and msg.header.frame_id != self._rigid_name:
            return
        q      = msg.pose.orientation
        ot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if not self._opt_valid:
            self._opt_origin_yaw = ot_yaw
            self._opt_valid      = True
            self.get_logger().info(
                f'OptiTrack listo  frame={msg.header.frame_id}  '
                f'yaw0={math.degrees(ot_yaw):.1f}°')
            return
        self._opt_yaw = math.atan2(
            math.sin(ot_yaw - self._opt_origin_yaw),
            math.cos(ot_yaw - self._opt_origin_yaw))

    def _cb_imu(self, msg: Imu) -> None:
        now = self._now_s()
        wz  = msg.angular_velocity.z
        if self._state in ('SPIN', 'COAST') and self._imu_last_t >= 0.0:
            dt = now - self._imu_last_t
            if 0.001 < dt < 0.2:
                self._imu_yaw += wz * dt
        self._imu_last_t = now

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _opt_dyaw(self) -> float:
        return math.atan2(
            math.sin(self._opt_yaw - self._opt_spin_start),
            math.cos(self._opt_yaw - self._opt_spin_start))

    def _kf_dyaw(self) -> float:
        return math.atan2(
            math.sin(self._kf.yaw - self._kf_spin_start),
            math.cos(self._kf.yaw - self._kf_spin_start))

    def _next_state(self, new_state: str) -> None:
        self._state       = new_state
        self._phase_start = self.get_clock().now()
        self._kf_last_t   = -1.0
        if new_state == 'SPIN':
            self._imu_yaw        = 0.0
            self._imu_last_t     = -1.0
            self._opt_spin_start = self._opt_yaw
            self._kf_spin_start  = self._kf.yaw
            self._spinup_detected = False
            self._spinup_delay_s  = 0.0
            self._spin_start_t    = self._now_s()
        elif new_state in ('COAST', 'PAUSE'):
            pass   # IMU sigue integrando en COAST

    def _csv_row(self, cmd_wz: float, event: str = '') -> None:
        od = self._opt_dyaw()
        kd = self._kf_dyaw()
        self._csv_w.writerow([
            f'{self._now_s():.3f}',
            self._rep, self._state, f'{self._elapsed():.3f}',
            f'{cmd_wz:.4f}',
            f'{math.degrees(od):.2f}',
            f'{math.degrees(self._imu_yaw):.2f}',
            f'{math.degrees(kd):.2f}',
            f'{math.degrees(kd - od):.2f}',
            event,
        ])
        if event:
            self._csv_file.flush()

    # ── Loop principal ────────────────────────────────────────────────────────

    def _loop(self) -> None:
        cmd_wz  = 0.0
        elapsed = self._elapsed()
        now_s   = self._now_s()

        # KF predict con comando del tick anterior
        if self._kf_last_t >= 0.0:
            dt = now_s - self._kf_last_t
            if 0.001 < dt < 0.2:
                self._kf.predict(self._prev_cmd_wz, dt)
        self._kf_last_t = now_s

        if self._state == 'WAIT':
            if self._opt_valid:
                self.get_logger().info(
                    f'Iniciando repetición 1/{self._n_reps}  '
                    f'w_cmd={self._w_cmd:.2f} rad/s  t_spin={self._t_spin:.1f}s')
                self._next_state('SPIN')

        elif self._state == 'SPIN':
            # Detectar spin-up: primera vez que opt_dyaw supera 2°
            od = self._opt_dyaw()
            if not self._spinup_detected and abs(od) > math.radians(2.0):
                self._spinup_delay_s  = now_s - self._spin_start_t
                self._spinup_detected = True

            if elapsed >= self._t_spin:
                # Guardar resultado de este giro
                od   = self._opt_dyaw()
                imu  = self._imu_yaw
                kd   = self._kf_dyaw()
                teor = self._w_cmd * self._t_spin                       # ángulo teórico
                factor = abs(od) / teor if teor > 0 else 0.0
                kf_err = math.degrees(kd - od)
                self._results.append({
                    'rep':          self._rep,
                    'opt_deg':      math.degrees(od),
                    'imu_deg':      math.degrees(imu),
                    'kf_deg':       math.degrees(kd),
                    'kf_err_deg':   kf_err,
                    'angle_factor': factor,
                    'spinup_s':     self._spinup_delay_s if self._spinup_detected else -1.0,
                })
                self.get_logger().info(
                    f'[rep {self._rep + 1}]  '
                    f'opt={math.degrees(od):+.1f}°  '
                    f'imu={math.degrees(imu):+.1f}°  '
                    f'kf={math.degrees(kd):+.1f}°  '
                    f'kf_err={kf_err:+.1f}°  '
                    f'factor={factor:.3f}  '
                    f'spinup={self._spinup_delay_s:.2f}s'
                    if self._spinup_detected else
                    f'[rep {self._rep + 1}]  '
                    f'opt={math.degrees(od):+.1f}°  factor={factor:.3f}  '
                    f'spinup=no detectado'
                )
                self._csv_row(0.0, f'FIN_SPIN_rep{self._rep}')
                self._next_state('COAST')
            else:
                cmd_wz = self._w_cmd + elapsed * 1e-6  # dither mínimo

        elif self._state == 'COAST':
            # Seguir registrando para medir overshoot
            if elapsed >= self._t_coast:
                od_coast = self._opt_dyaw()
                od_spin  = self._results[-1]['opt_deg']
                overshoot = math.degrees(od_coast) - od_spin
                self._results[-1]['overshoot_deg'] = overshoot
                self.get_logger().info(
                    f'  inercia: opt_final={math.degrees(od_coast):.1f}°  '
                    f'overshoot={overshoot:+.1f}°')
                self._csv_row(0.0, f'FIN_COAST_rep{self._rep}')
                self._rep += 1
                if self._rep >= self._n_reps:
                    self._print_summary()
                    self._state = 'DONE'
                else:
                    self.get_logger().info(
                        f'Pausa {self._t_pause:.0f}s → repetición '
                        f'{self._rep + 1}/{self._n_reps}')
                    self._next_state('PAUSE')

        elif self._state == 'PAUSE':
            if elapsed >= self._t_pause:
                self.get_logger().info(
                    f'Iniciando repetición {self._rep + 1}/{self._n_reps}')
                self._next_state('SPIN')

        cmd = Twist()
        cmd.angular.z = cmd_wz
        self._pub_vel.publish(cmd)
        self._prev_cmd_wz = cmd_wz
        self._csv_row(cmd_wz)

    # ── Resumen final ─────────────────────────────────────────────────────────

    def _print_summary(self) -> None:
        n = len(self._results)
        if n == 0:
            return

        opts      = [r['opt_deg']      for r in self._results]
        imus      = [r['imu_deg']      for r in self._results]
        kfs       = [r['kf_deg']       for r in self._results]
        kf_errs   = [r['kf_err_deg']   for r in self._results]
        factors   = [r['angle_factor'] for r in self._results]
        overshoots= [r.get('overshoot_deg', 0.0) for r in self._results]
        spinups   = [r['spinup_s'] for r in self._results if r['spinup_s'] >= 0]

        teor = math.degrees(self._w_cmd * self._t_spin)
        sep  = '=' * 62

        self.get_logger().info(sep)
        self.get_logger().info(
            f'RESUMEN  n={n}  w_cmd={self._w_cmd:.2f} rad/s  t_spin={self._t_spin:.1f}s')
        self.get_logger().info(
            f'  Teórico:       {teor:.1f}°  (= w × t)')
        self.get_logger().info(
            f'  opt_dyaw:      '
            f'μ={statistics.mean(opts):.1f}°  '
            f'σ={statistics.stdev(opts) if n > 1 else 0:.1f}°  '
            f'[{min(opts):.1f}°, {max(opts):.1f}°]')
        self.get_logger().info(
            f'  imu_yaw:       '
            f'μ={statistics.mean(imus):.1f}°  '
            f'σ={statistics.stdev(imus) if n > 1 else 0:.1f}°')
        self.get_logger().info(
            f'  kf_dyaw:       '
            f'μ={statistics.mean(kfs):.1f}°  '
            f'σ={statistics.stdev(kfs) if n > 1 else 0:.1f}°')
        self.get_logger().info(
            f'  kf_error:      '
            f'μ={statistics.mean(kf_errs):+.1f}°  '
            f'σ={statistics.stdev(kf_errs) if n > 1 else 0:.1f}°  '
            f'(kf - opt, >0 → KF sobreestima)')
        self.get_logger().info(
            f'  overshoot:     '
            f'μ={statistics.mean(overshoots):+.1f}°  '
            f'σ={statistics.stdev(overshoots) if n > 1 else 0:.1f}°  '
            f'(inercia tras cmd=0)')
        if spinups:
            self.get_logger().info(
                f'  spin-up delay: '
                f'μ={statistics.mean(spinups):.2f}s  '
                f'σ={statistics.stdev(spinups) if len(spinups) > 1 else 0:.2f}s')
        self.get_logger().info(
            f'  angle_factor sugerido: {statistics.mean(factors):.4f}  '
            f'(actual/teórico — usar en calibration.yaml)')
        self.get_logger().info(sep)
        self.get_logger().info(f'CSV: {self._log_path}')

        # Guardar sugerencia de calibración
        out = os.path.expanduser('~/ros2_ws/spin_calibration_suggestion.yaml')
        mean_factor  = statistics.mean(factors)
        mean_spinup  = statistics.mean(spinups) if spinups else 0.0
        mean_over    = statistics.mean(overshoots)
        with open(out, 'w') as f:
            f.write('# Sugerencia de calibración de giros\n')
            f.write(f'# w_cmd={self._w_cmd:.2f}  t_spin={self._t_spin:.1f}s  n={n}\n')
            f.write(f'# opt_media={statistics.mean(opts):.1f}°  teórico={teor:.1f}°\n')
            f.write('angle:\n')
            f.write(f'  correction_factor: {mean_factor:.4f}\n')
            f.write(f'  # spin_up_delay_s: {mean_spinup:.2f}\n')
            f.write(f'  # overshoot_deg:   {mean_over:.1f}\n')
            # Sugerir ajuste de spin_kp y spin_w_min para drift_square_robot
            f.write(f'# Para drift_square_robot:\n')
            f.write(f'#   spin_kp:    2.5   # sin cambios necesarios con P controller\n')
            f.write(f'#   spin_w_min: {max(0.2, abs(mean_over) * 0.1):.2f}  '
                    f'# reducir si hay overshoot excesivo\n')
        self.get_logger().info(f'Calibración guardada: {out}')

        self._csv_file.flush()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpinCharacterizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node._pub_vel.publish(stop)
        node._csv_file.flush()
        node._csv_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
