"""
Navegador de 4 waypoints usando OptiTrack como feedback de posición real.

Protecciones de seguridad:
  - Datos OptiTrack congelados >0.5s → frena y entra en STALE
  - Distancia al waypoint creciendo por >3s → aborta ese waypoint
  - Timeout por waypoint (default 30s) → aborta y pasa al siguiente
  - Distancia máxima desde el origen (default 3m) → frena todo

Topics:
  /optitrack/rigid_body  ← PoseStamped  (feedback de posición real)
  /cmd_vel               → Twist        (comandos de velocidad)

Parámetros ROS2:
  cmd_vel_topic  : tópico de velocidad        (default: /cmd_vel)
  wp_side        : lado del cuadrado [m]      (default: 0.8)
  v_max          : vel. lineal máxima         (default: 0.15)
  w_max          : vel. angular máxima        (default: 0.8)
  k_v            : ganancia lineal            (default: 0.35)
  k_w            : ganancia angular           (default: 1.4)
  dist_tol       : tolerancia posición [m]    (default: 0.08)
  heading_tol    : tolerancia ángulo [rad]    (default: 0.12)
  stale_timeout  : timeout dato congelado [s] (default: 0.5)
  wp_timeout     : timeout por waypoint [s]   (default: 30.0)
  max_dist       : distancia máx desde origen (default: 3.0)
  log_dir        : directorio de logs         (default: /home/jetson/logs)
"""

import csv
import json
import math
import os
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

_OPTI_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_CMD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


class OptitrackNavigator(Node):

    _REL_WAYPOINTS = [
        (0.8,  0.0),
        (0.8,  0.8),
        (0.0,  0.8),
        (0.0,  0.0),
    ]

    def __init__(self) -> None:
        super().__init__('optitrack_navigator')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wp_side',       0.8)
        self.declare_parameter('v_max',         0.15)
        self.declare_parameter('w_max',         0.80)
        self.declare_parameter('k_v',           0.35)
        self.declare_parameter('k_w',           1.40)
        self.declare_parameter('dist_tol',      0.08)
        self.declare_parameter('heading_tol',   0.12)
        self.declare_parameter('stale_timeout', 0.5)
        self.declare_parameter('wp_timeout',    30.0)
        self.declare_parameter('max_dist',      3.0)
        self.declare_parameter('log_dir',       '/home/jetson/logs')

        cmd_topic          = self.get_parameter('cmd_vel_topic').value
        side               = self.get_parameter('wp_side').value
        self._v_max        = self.get_parameter('v_max').value
        self._w_max        = self.get_parameter('w_max').value
        self._k_v          = self.get_parameter('k_v').value
        self._k_w          = self.get_parameter('k_w').value
        self._dist_tol     = self.get_parameter('dist_tol').value
        self._hdg_tol      = self.get_parameter('heading_tol').value
        self._stale_timeout= self.get_parameter('stale_timeout').value
        self._wp_timeout   = self.get_parameter('wp_timeout').value
        self._max_dist     = self.get_parameter('max_dist').value
        log_dir            = self.get_parameter('log_dir').value

        self._rel_wps = [
            (x * side / 0.8, y * side / 0.8) for x, y in self._REL_WAYPOINTS
        ]

        # --- Estado de posición ---
        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0
        self._last_pose_time: float | None = None   # tiempo del último mensaje OptiTrack
        self._last_x = 0.0
        self._last_y = 0.0

        # --- Estado de misión ---
        self._origin_x:   float | None = None
        self._origin_y:   float | None = None
        self._wps:        list[tuple[float, float]] = []
        self._wp_idx      = 0
        self._done        = False
        self._phase       = 'WAIT'   # WAIT / ROTATING / DRIVING / STALE / ABORT / DONE
        self._t0:         float | None = None
        self._wp_t0:      float | None = None
        self._log_counter = 0

        # --- Anti-divergencia: seguimiento de distancia al WP ---
        self._dist_prev      = float('inf')
        self._dist_grow_since: float | None = None   # cuándo empezó a crecer

        # --- Logging ---
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path  = os.path.join(log_dir, f'nav_{ts}.csv')
        self._json_path = os.path.join(log_dir, f'nav_{ts}_summary.json')
        self._log_rows:   list[list] = []
        self._wp_events:  list[dict] = []
        self._abort_log:  list[str]  = []

        # --- Pub/Sub ---
        self.create_subscription(
            PoseStamped, '/optitrack/rigid_body', self._opti_cb, _OPTI_QOS
        )
        self._pub_vel = self.create_publisher(Twist, cmd_topic, _CMD_QOS)

        self.create_timer(0.05, self._control_loop)   # 20 Hz

        self.get_logger().info(
            f'OptiTrack Navigator listo.\n'
            f'  Cuadrado: {side}m x {side}m\n'
            f'  tol_dist={self._dist_tol*100:.0f}cm  '
            f'tol_hdg={math.degrees(self._hdg_tol):.0f}°\n'
            f'  stale_timeout={self._stale_timeout}s  '
            f'wp_timeout={self._wp_timeout}s  '
            f'max_dist={self._max_dist}m\n'
            f'  CSV → {self._csv_path}'
        )

    # ------------------------------------------------------------------
    def _opti_cb(self, msg: PoseStamped) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self._x   = msg.pose.position.x
        self._y   = msg.pose.position.y
        q = msg.pose.orientation
        self._yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        self._last_pose_time = now

        if self._origin_x is None:
            self._origin_x = self._x
            self._origin_y = self._y
            self._t0    = now
            self._wp_t0 = now

            self._wps = [
                (self._origin_x + dx, self._origin_y + dy)
                for dx, dy in self._rel_wps
            ]
            self._phase = 'ROTATING'

            self.get_logger().warn(
                f'ORIGEN fijado: x={self._origin_x:.3f}  '
                f'y={self._origin_y:.3f}  '
                f'yaw={math.degrees(self._yaw):.1f}°\n'
                + '\n'.join(
                    f'  WP{i}: ({wx:.3f}, {wy:.3f})'
                    for i, (wx, wy) in enumerate(self._wps)
                )
            )

    # ------------------------------------------------------------------
    def _stop(self) -> None:
        self._pub_vel.publish(Twist())

    def _check_safety(self, now: float) -> bool:
        """
        Retorna True si hay que abortar el ciclo de control por seguridad.
        Frena el robot en cada caso.
        """
        # 1. Datos OptiTrack congelados
        if self._last_pose_time is not None:
            staleness = now - self._last_pose_time
            if staleness > self._stale_timeout:
                if self._phase not in ('STALE', 'DONE', 'WAIT'):
                    self._phase = 'STALE'
                    msg = (
                        f'STALE: OptiTrack sin datos por {staleness:.2f}s — '
                        'robot detenido. Posible choque o salida del área.'
                    )
                    self.get_logger().error(msg)
                    self._abort_log.append(msg)
                    self._stop()
                return True

        # 2. Distancia máxima desde origen superada
        if self._origin_x is not None:
            dist_from_origin = math.hypot(
                self._x - self._origin_x, self._y - self._origin_y
            )
            if dist_from_origin > self._max_dist:
                if self._phase not in ('ABORT', 'DONE'):
                    self._phase = 'ABORT'
                    msg = (
                        f'ABORT: robot a {dist_from_origin:.2f}m del origen '
                        f'(máx={self._max_dist}m) — detenido.'
                    )
                    self.get_logger().error(msg)
                    self._abort_log.append(msg)
                    self._stop()
                    self._save_logs(now - self._t0)
                return True

        # 3. Timeout por waypoint
        if self._wp_t0 is not None and self._phase in ('ROTATING', 'DRIVING'):
            wp_elapsed = now - self._wp_t0
            if wp_elapsed > self._wp_timeout:
                msg = (
                    f'TIMEOUT WP{self._wp_idx} ({wp_elapsed:.1f}s > '
                    f'{self._wp_timeout}s) — saltando al siguiente.'
                )
                self.get_logger().warn(msg)
                self._abort_log.append(msg)
                self._wp_events.append({
                    'waypoint':       self._wp_idx,
                    'target_x':       round(self._wps[self._wp_idx][0], 4),
                    'target_y':       round(self._wps[self._wp_idx][1], 4),
                    'actual_x':       round(self._x, 4),
                    'actual_y':       round(self._y, 4),
                    'pos_error_m':    None,
                    'pos_error_cm':   None,
                    'heading_error_deg': None,
                    'time_to_wp_s':   round(wp_elapsed, 2),
                    'resultado':      'TIMEOUT',
                })
                self._wp_idx += 1
                self._wp_t0  = now
                self._dist_prev      = float('inf')
                self._dist_grow_since = None
                if self._wp_idx >= len(self._wps):
                    self._done  = True
                    self._phase = 'DONE'
                    self._stop()
                    self._save_logs(now - self._t0)
                    return True
                self._phase = 'ROTATING'
                return False   # continúa con el nuevo WP

        return False

    # ------------------------------------------------------------------
    def _control_loop(self) -> None:
        cmd = Twist()
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._phase == 'WAIT' or self._origin_x is None:
            self._pub_vel.publish(cmd)
            return

        if self._phase in ('DONE', 'ABORT'):
            self._pub_vel.publish(cmd)
            return

        # Recuperación de STALE: si vuelven datos, reanudar
        if self._phase == 'STALE':
            if self._last_pose_time and (now - self._last_pose_time) < self._stale_timeout:
                self.get_logger().warn('OptiTrack recuperado — reanudando.')
                self._phase  = 'ROTATING'
                self._wp_t0  = now   # reiniciar timeout del WP
            else:
                self._pub_vel.publish(cmd)
                return

        if _check := self._check_safety(now):
            return

        wp_x, wp_y = self._wps[self._wp_idx]
        err_x  = wp_x - self._x
        err_y  = wp_y - self._y
        dist   = math.hypot(err_x, err_y)
        target_th = math.atan2(err_y, err_x)
        err_th = _wrap(target_th - self._yaw)
        elapsed = now - self._t0

        # Detectar si la distancia al WP crece sostenidamente (robot alejándose)
        if dist > self._dist_prev + 0.02:   # creció más de 2cm
            if self._dist_grow_since is None:
                self._dist_grow_since = now
            elif now - self._dist_grow_since > 3.0 and self._phase == 'DRIVING':
                msg = (
                    f'WP{self._wp_idx}: distancia creciendo por '
                    f'{now - self._dist_grow_since:.1f}s '
                    f'(ahora {dist*100:.1f}cm) — frenando y re-orientando.'
                )
                self.get_logger().warn(msg)
                self._abort_log.append(msg)
                self._phase = 'ROTATING'
                self._dist_grow_since = None
        else:
            self._dist_grow_since = None
        self._dist_prev = dist

        # ── Máquina de estados ─────────────────────────────────────────
        if self._phase == 'ROTATING':
            cmd.angular.z = _clamp(self._k_w * err_th, -self._w_max, self._w_max)
            if abs(err_th) < self._hdg_tol:
                self._phase = 'DRIVING'
                self._dist_grow_since = None
                self._dist_prev = dist
                self.get_logger().info(
                    f'WP{self._wp_idx}: alineado '
                    f'(err_hdg={math.degrees(err_th):.1f}°) → DRIVING'
                )

        elif self._phase == 'DRIVING':
            if dist < self._dist_tol:
                self._stop()
                wp_elapsed = now - self._wp_t0
                self._log_wp_reached(self._wp_idx, wp_x, wp_y, dist, err_th, wp_elapsed)
                self._wp_idx += 1
                self._wp_t0  = now
                self._dist_prev       = float('inf')
                self._dist_grow_since = None

                if self._wp_idx >= len(self._wps):
                    self._done  = True
                    self._phase = 'DONE'
                    self.get_logger().warn(
                        f'=== TRAYECTORIA COMPLETA en {elapsed:.1f}s ==='
                    )
                    self._save_logs(elapsed)
                else:
                    self._phase = 'ROTATING'
                    self.get_logger().info(
                        f'WP{self._wp_idx - 1} ALCANZADO → '
                        f'girando hacia WP{self._wp_idx}'
                    )
                return

            align = math.cos(err_th)
            cmd.linear.x  = _clamp(
                self._k_v * dist * max(align, 0.0), 0.0, self._v_max
            )
            cmd.angular.z = _clamp(self._k_w * err_th, -self._w_max, self._w_max)

            if abs(err_th) > math.pi / 2.5:
                self._phase = 'ROTATING'
                self.get_logger().warn(
                    f'WP{self._wp_idx}: heading muy desviado '
                    f'({math.degrees(err_th):.1f}°) → re-orientando'
                )

        # ── Log periódico ──────────────────────────────────────────────
        self._log_counter += 1
        if self._log_counter >= 4:
            self._log_counter = 0
            self.get_logger().info(
                f'[{elapsed:6.1f}s] WP{self._wp_idx}  '
                f'pos=({self._x:.3f},{self._y:.3f})  '
                f'dist={dist*100:.1f}cm  '
                f'hdg={math.degrees(err_th):.1f}°  '
                f'v={cmd.linear.x:.3f}  w={cmd.angular.z:.3f}  '
                f'{self._phase}'
            )

        self._log_rows.append([
            round(elapsed, 3),
            round(self._x, 4),
            round(self._y, 4),
            round(math.degrees(self._yaw), 2),
            self._wp_idx,
            round(wp_x, 4),
            round(wp_y, 4),
            round(dist, 4),
            round(math.degrees(err_th), 2),
            round(cmd.linear.x, 4),
            round(cmd.angular.z, 4),
            self._phase,
        ])

        self._pub_vel.publish(cmd)

    # ------------------------------------------------------------------
    def _log_wp_reached(self, idx, wp_x, wp_y, dist_err, hdg_err, elapsed_wp):
        self._wp_events.append({
            'waypoint':          idx,
            'target_x':          round(wp_x, 4),
            'target_y':          round(wp_y, 4),
            'actual_x':          round(self._x, 4),
            'actual_y':          round(self._y, 4),
            'pos_error_m':       round(dist_err, 4),
            'pos_error_cm':      round(dist_err * 100, 2),
            'heading_error_deg': round(math.degrees(hdg_err), 2),
            'time_to_wp_s':      round(elapsed_wp, 2),
            'resultado':         'ALCANZADO',
        })
        self.get_logger().warn(
            f'✓ WP{idx} ALCANZADO  '
            f'target=({wp_x:.3f},{wp_y:.3f})  '
            f'actual=({self._x:.3f},{self._y:.3f})  '
            f'error={dist_err*100:.1f}cm  '
            f'tiempo={elapsed_wp:.1f}s'
        )

    # ------------------------------------------------------------------
    def _save_logs(self, total_time: float) -> None:
        try:
            with open(self._csv_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow([
                    'tiempo_s', 'x', 'y', 'yaw_deg',
                    'wp_idx', 'wp_x', 'wp_y',
                    'dist_wp_m', 'hdg_err_deg',
                    'cmd_v', 'cmd_w', 'fase',
                ])
                w.writerows(self._log_rows)
            self.get_logger().info(f'CSV guardado: {self._csv_path}')
        except Exception as e:
            self.get_logger().error(f'Error CSV: {e}')

        errors_cm = [
            ev['pos_error_cm'] for ev in self._wp_events
            if ev.get('pos_error_cm') is not None
        ]
        diagnosis = self._diagnose()
        summary = {
            'fecha':           datetime.now().isoformat(),
            'tiempo_total_s':  round(total_time, 2),
            'n_waypoints':     len(self._wps),
            'waypoint_events': self._wp_events,
            'abort_log':       self._abort_log,
            'estadisticas': {
                'wps_alcanzados':     sum(1 for ev in self._wp_events if ev.get('resultado') == 'ALCANZADO'),
                'wps_timeout':        sum(1 for ev in self._wp_events if ev.get('resultado') == 'TIMEOUT'),
                'error_promedio_cm':  round(sum(errors_cm) / len(errors_cm), 2) if errors_cm else None,
                'error_maximo_cm':    round(max(errors_cm), 2) if errors_cm else None,
            },
            'diagnostico':     diagnosis,
            'parametros': {
                'v_max':           self._v_max,
                'w_max':           self._w_max,
                'k_v':             self._k_v,
                'k_w':             self._k_w,
                'dist_tol_m':      self._dist_tol,
                'hdg_tol_deg':     round(math.degrees(self._hdg_tol), 1),
                'stale_timeout_s': self._stale_timeout,
                'wp_timeout_s':    self._wp_timeout,
                'max_dist_m':      self._max_dist,
            },
        }
        try:
            with open(self._json_path, 'w') as f:
                json.dump(summary, f, indent=2, ensure_ascii=False)
            self.get_logger().warn(f'Diagnóstico guardado: {self._json_path}')
        except Exception as e:
            self.get_logger().error(f'Error JSON: {e}')

        self.get_logger().warn('=' * 55)
        self.get_logger().warn('RESUMEN')
        for ev in self._wp_events:
            err = f"{ev['pos_error_cm']:.1f}cm" if ev.get('pos_error_cm') else 'N/A'
            self.get_logger().warn(
                f"  WP{ev['waypoint']}: {ev['resultado']}  "
                f"error={err}  tiempo={ev['time_to_wp_s']:.1f}s"
            )
        if self._abort_log:
            self.get_logger().warn('INCIDENCIAS:')
            for a in self._abort_log:
                self.get_logger().warn(f'  ! {a}')
        self.get_logger().warn('DIAGNOSTICO:')
        for d in diagnosis:
            self.get_logger().warn(f'  → {d}')
        self.get_logger().warn('=' * 55)

    # ------------------------------------------------------------------
    def _diagnose(self) -> list[str]:
        diag = []
        if not self._wp_events:
            return ['No se completó ningún waypoint']

        timeouts = sum(1 for ev in self._wp_events if ev.get('resultado') == 'TIMEOUT')
        if timeouts:
            diag.append(
                f'{timeouts} waypoint(s) por timeout: '
                'aumentar wp_timeout o reducir wp_side'
            )

        stale_events = [a for a in self._abort_log if 'STALE' in a]
        if stale_events:
            diag.append(
                'Robot perdió visibilidad del OptiTrack (choque o salida del área). '
                'Reducir velocidad v_max o el tamaño del cuadrado wp_side.'
            )

        grow_events = [a for a in self._abort_log if 'creciendo' in a]
        if grow_events:
            diag.append(
                'Distancia al waypoint creció — el robot se alejó. '
                'Verificar que cmd_vel llega al hardware o revisar ejes del robot.'
            )

        errors_cm = [
            ev['pos_error_cm'] for ev in self._wp_events
            if ev.get('pos_error_cm') is not None
        ]
        if errors_cm:
            avg = sum(errors_cm) / len(errors_cm)
            if avg > 15:
                diag.append(
                    f'Error de posición alto ({avg:.1f}cm promedio): '
                    'reducir v_max para frenado más suave'
                )
            elif avg < 8:
                diag.append(f'Buena precisión ({avg:.1f}cm promedio) ✓')
            else:
                diag.append(f'Precisión aceptable ({avg:.1f}cm promedio)')

        if not diag:
            diag.append('Sin incidencias registradas')
        return diag


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OptitrackNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node._done and node._log_rows:
            node.get_logger().warn('Interrumpido — guardando log parcial...')
            elapsed = (
                node.get_clock().now().nanoseconds * 1e-9 - node._t0
                if node._t0 else 0.0
            )
            node._save_logs(elapsed)
        node._stop()
        node.get_logger().info('Robot detenido.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
