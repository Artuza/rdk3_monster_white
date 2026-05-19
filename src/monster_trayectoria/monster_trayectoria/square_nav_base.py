"""
Base compartida para square_nav_optitrack.py y square_nav_odom.py.

Control idéntico en ambos nodos — solo cambia la fuente de posición.

Parámetros ROS2:
  cmd_vel_topic  : tópico de velocidad        (default: /cmd_vel)
  wp_side        : lado del cuadrado [m]      (default: 0.8)
  v_max          : vel. lineal máxima [m/s]   (default: 0.15)
  w_max          : vel. angular máxima [r/s]  (default: 0.80)
  k_v            : ganancia lineal P          (default: 0.35)
  k_w            : ganancia angular P         (default: 1.40)
  dist_tol       : tolerancia de llegada [m]  (default: 0.08)
  heading_tol    : tolerancia de giro [rad]   (default: 0.12)
  stale_timeout  : datos congelados → stop[s] (default: 0.5)
  wp_timeout     : timeout por waypoint [s]   (default: 30.0)
  max_dist       : radio máximo desde origen  (default: 3.0)
  log_dir        : directorio de logs         (default: /home/jetson/logs)

Fases de control (por waypoint):
  WAIT      → esperando primer mensaje de posición
  ROTATING  → giro en sitio hasta apuntar al WP (< heading_tol)
  DRIVING   → avance + corrección angular hasta llegar (< dist_tol)
  STALE     → datos congelados, robot frenado
  DONE      → trayectoria completa
  ABORT     → robot fuera del área permitida
"""

import csv
import json
import math
import os
from datetime import datetime

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

_CMD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# Waypoints relativos al origen (escalados por wp_side/0.8)
_REL_WPS = [
    (0.8,  0.0),   # WP0: adelante
    (0.8,  0.8),   # WP1: diagonal
    (0.0,  0.8),   # WP2: lateral
    (0.0,  0.0),   # WP3: regreso al origen
]


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class SquareNavigatorBase(Node):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        # ── Parámetros ────────────────────────────────────────────────
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wp_side',       0.8)
        self.declare_parameter('v_max',         0.15)
        self.declare_parameter('w_max',         0.80)
        self.declare_parameter('k_v',           0.35)
        self.declare_parameter('k_w',           1.40)
        self.declare_parameter('dist_tol',      0.08)
        self.declare_parameter('heading_tol',   0.12)
        self.declare_parameter('w_min',         0.45)
        self.declare_parameter('v_min',         0.07)
        self.declare_parameter('v_rot',         0.06)   # vel. lineal mínima al girar
        self.declare_parameter('stale_timeout', 0.5)
        self.declare_parameter('wp_timeout',    30.0)
        self.declare_parameter('max_dist',      3.0)
        self.declare_parameter('log_dir',       '/home/jetson/logs')

        cmd_topic           = self.get_parameter('cmd_vel_topic').value
        side                = float(self.get_parameter('wp_side').value)
        self._v_max         = float(self.get_parameter('v_max').value)
        self._w_max         = float(self.get_parameter('w_max').value)
        self._k_v           = float(self.get_parameter('k_v').value)
        self._k_w           = float(self.get_parameter('k_w').value)
        self._dist_tol      = float(self.get_parameter('dist_tol').value)
        self._hdg_tol       = float(self.get_parameter('heading_tol').value)
        self._w_min         = float(self.get_parameter('w_min').value)
        self._v_min         = float(self.get_parameter('v_min').value)
        self._v_rot         = float(self.get_parameter('v_rot').value)
        self._stale_timeout = float(self.get_parameter('stale_timeout').value)
        self._wp_timeout    = float(self.get_parameter('wp_timeout').value)
        self._max_dist      = float(self.get_parameter('max_dist').value)
        log_dir             = self.get_parameter('log_dir').value

        self._side = side
        self._rel_wps = [(x * side / 0.8, y * side / 0.8) for x, y in _REL_WPS]

        # ── Estado de posición ────────────────────────────────────────
        self._x:    float = 0.0
        self._y:    float = 0.0
        self._yaw:  float = 0.0
        self._last_pose_time: float | None = None

        # ── Estado de misión ──────────────────────────────────────────
        self._origin_x: float | None = None
        self._origin_y: float | None = None
        self._wps:      list[tuple[float, float]] = []
        self._wp_idx:   int   = 0
        self._phase:    str   = 'WAIT'
        self._done:     bool  = False
        self._t0:       float | None = None
        self._wp_t0:    float | None = None

        # ── Anti-divergencia ──────────────────────────────────────────
        self._dist_prev:       float        = float('inf')
        self._dist_grow_since: float | None = None

        # ── Logging ───────────────────────────────────────────────────
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path  = os.path.join(log_dir, f'{node_name}_{ts}.csv')
        self._json_path = os.path.join(log_dir, f'{node_name}_{ts}_summary.json')
        self._log_rows:  list[list] = []
        self._wp_events: list[dict] = []
        self._incidents: list[str]  = []
        self._log_tick  = 0

        # ── Pub / Timer ───────────────────────────────────────────────
        self._pub_vel = self.create_publisher(Twist, cmd_topic, _CMD_QOS)
        self.create_timer(0.05, self._control_loop)  # 20 Hz

    # ── Banner de inicio ──────────────────────────────────────────────
    def _startup_banner(self) -> str:
        return (
            f'  cuadrado={self._side}m  '
            f'v_max={self._v_max}  w_max={self._w_max}\n'
            f'  k_v={self._k_v}  k_w={self._k_w}\n'
            f'  dist_tol={self._dist_tol*100:.0f}cm  '
            f'hdg_tol={math.degrees(self._hdg_tol):.0f}°\n'
            f'  w_min={self._w_min}  v_rot={self._v_rot}  v_min={self._v_min}\n'
            f'  stale={self._stale_timeout}s  '
            f'wp_timeout={self._wp_timeout}s  '
            f'max_dist={self._max_dist}m\n'
            f'  CSV → {self._csv_path}'
        )

    # ── Llamado por la subclase cada vez que llega una pose ───────────
    def _update_pose(self, x: float, y: float, yaw: float) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self._x   = x
        self._y   = y
        self._yaw = yaw
        self._last_pose_time = now

        if self._origin_x is None:
            self._origin_x = x
            self._origin_y = y
            self._t0    = now
            self._wp_t0 = now
            self._wps   = [
                (self._origin_x + dx, self._origin_y + dy)
                for dx, dy in self._rel_wps
            ]
            self._phase = 'ROTATING'
            self.get_logger().warn(
                f'ORIGEN fijado: x={x:.3f}  y={y:.3f}  yaw={math.degrees(yaw):.1f}°\n'
                + '\n'.join(
                    f'  WP{i}: ({wx:.3f}, {wy:.3f})'
                    for i, (wx, wy) in enumerate(self._wps)
                )
            )

    # ── Loop de control (20 Hz) ───────────────────────────────────────
    def _control_loop(self) -> None:
        cmd  = Twist()
        now  = self.get_clock().now().nanoseconds * 1e-9

        # Fases terminales
        if self._phase in ('WAIT', 'DONE', 'ABORT') or self._origin_x is None:
            self._pub_vel.publish(cmd)
            return

        # ── Seguridad: datos congelados ───────────────────────────────
        if self._last_pose_time is not None:
            stale = now - self._last_pose_time
            if stale > self._stale_timeout:
                if self._phase != 'STALE':
                    self._phase = 'STALE'
                    msg = (
                        f'STALE: sin datos {stale:.2f}s — robot detenido. '
                        'Posible choque o salida del área de captura.'
                    )
                    self.get_logger().error(msg)
                    self._incidents.append(msg)
                self._pub_vel.publish(cmd)
                return

        # Recuperación de STALE
        if self._phase == 'STALE':
            self._phase  = 'ROTATING'
            self._wp_t0  = now
            self.get_logger().warn('Datos recuperados — reanudando.')

        # ── Seguridad: distancia máxima ───────────────────────────────
        dist_origin = math.hypot(
            self._x - self._origin_x, self._y - self._origin_y
        )
        if dist_origin > self._max_dist:
            self._phase = 'ABORT'
            msg = (
                f'ABORT: robot a {dist_origin:.2f}m del origen '
                f'(máx={self._max_dist}m) — detenido.'
            )
            self.get_logger().error(msg)
            self._incidents.append(msg)
            self._pub_vel.publish(cmd)
            self._save_logs(now - self._t0)
            return

        # ── Seguridad: timeout por waypoint ──────────────────────────
        if self._wp_t0 is not None:
            wp_elapsed = now - self._wp_t0
            if wp_elapsed > self._wp_timeout:
                msg = (
                    f'TIMEOUT WP{self._wp_idx} '
                    f'({wp_elapsed:.1f}s) — saltando.'
                )
                self.get_logger().warn(msg)
                self._incidents.append(msg)
                self._record_wp_event('TIMEOUT')
                self._advance_wp(now)
                if self._done:
                    self._pub_vel.publish(cmd)
                    return

        # ── Variables de control ──────────────────────────────────────
        wp_x, wp_y = self._wps[self._wp_idx]
        err_x = wp_x - self._x
        err_y = wp_y - self._y
        dist  = math.hypot(err_x, err_y)
        target_th = math.atan2(err_y, err_x)
        err_th    = _wrap(target_th - self._yaw)
        elapsed   = now - self._t0

        # ── Detección de alejamiento ──────────────────────────────────
        if self._phase == 'DRIVING':
            if dist > self._dist_prev + 0.02:
                if self._dist_grow_since is None:
                    self._dist_grow_since = now
                elif now - self._dist_grow_since > 3.0:
                    msg = (
                        f'WP{self._wp_idx}: distancia creciendo '
                        f'{now - self._dist_grow_since:.1f}s '
                        f'({dist*100:.1f}cm) — re-orientando.'
                    )
                    self.get_logger().warn(msg)
                    self._incidents.append(msg)
                    self._phase = 'ROTATING'
                    self._dist_grow_since = None
            else:
                self._dist_grow_since = None
        self._dist_prev = dist

        # ── Máquina de estados ────────────────────────────────────────
        if self._phase == 'ROTATING':
            raw_w = self._k_w * err_th
            if abs(raw_w) > 0.01:
                raw_w = math.copysign(max(abs(raw_w), self._w_min), raw_w)
            cmd.angular.z = _clamp(raw_w, -self._w_max, self._w_max)
            # Pequeña velocidad forward para que el MCU acepte el comando angular
            if self._v_rot > 0.0:
                cmd.linear.x = self._v_rot
            if abs(err_th) < self._hdg_tol:
                self._phase = 'DRIVING'
                self._dist_prev = dist
                self._dist_grow_since = None
                self.get_logger().info(
                    f'WP{self._wp_idx}: alineado '
                    f'(err_hdg={math.degrees(err_th):.1f}°) → DRIVING'
                )

        elif self._phase == 'DRIVING':
            if dist < self._dist_tol:
                # Waypoint alcanzado
                self._pub_vel.publish(cmd)
                self._record_wp_event('ALCANZADO', dist, err_th, now)
                self._advance_wp(now)
                return

            align = math.cos(err_th)
            raw_v = self._k_v * dist * max(align, 0.0)
            # Aplicar mínimo para vencer fricción estática
            if raw_v > 0.01:
                raw_v = max(raw_v, self._v_min)
            cmd.linear.x  = _clamp(raw_v, 0.0, self._v_max)
            cmd.angular.z = _clamp(self._k_w * err_th, -self._w_max, self._w_max)

            # Si el heading se desvía mucho, volver a orientar
            if abs(err_th) > math.pi / 2.5:
                self._phase = 'ROTATING'
                self.get_logger().warn(
                    f'WP{self._wp_idx}: heading {math.degrees(err_th):.1f}° '
                    '— re-orientando'
                )

        # ── Log periódico (cada 0.2s) ─────────────────────────────────
        self._log_tick += 1
        if self._log_tick >= 4:
            self._log_tick = 0
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

    # ── Helpers ───────────────────────────────────────────────────────
    def _advance_wp(self, now: float) -> None:
        self._wp_idx += 1
        self._wp_t0   = now
        self._dist_prev       = float('inf')
        self._dist_grow_since = None

        if self._wp_idx >= len(self._wps):
            self._done  = True
            self._phase = 'DONE'
            elapsed = now - self._t0
            self.get_logger().warn(
                f'=== TRAYECTORIA COMPLETA en {elapsed:.1f}s ==='
            )
            self._save_logs(elapsed)
        else:
            self._phase = 'ROTATING'
            self.get_logger().info(
                f'→ WP{self._wp_idx} '
                f'({self._wps[self._wp_idx][0]:.3f}, '
                f'{self._wps[self._wp_idx][1]:.3f})'
            )

    def _record_wp_event(
        self,
        resultado: str,
        dist_err: float = 0.0,
        hdg_err:  float = 0.0,
        now:      float = 0.0,
    ) -> None:
        wp_x, wp_y = self._wps[self._wp_idx]
        elapsed_wp = (now - self._wp_t0) if now and self._wp_t0 else 0.0
        ev: dict = {
            'waypoint':  self._wp_idx,
            'resultado': resultado,
            'target':    (round(wp_x, 4), round(wp_y, 4)),
            'actual':    (round(self._x, 4), round(self._y, 4)),
            'pos_error_cm':      round(dist_err * 100, 2) if resultado == 'ALCANZADO' else None,
            'hdg_error_deg':     round(math.degrees(hdg_err), 2) if resultado == 'ALCANZADO' else None,
            'tiempo_s':          round(elapsed_wp, 2),
        }
        self._wp_events.append(ev)
        if resultado == 'ALCANZADO':
            self.get_logger().warn(
                f'✓ WP{self._wp_idx} ALCANZADO  '
                f'error={dist_err*100:.1f}cm  '
                f'tiempo={elapsed_wp:.1f}s'
            )

    # ── Guardar logs ──────────────────────────────────────────────────
    def _save_logs(self, total_time: float) -> None:
        # CSV
        try:
            import csv as _csv
            with open(self._csv_path, 'w', newline='') as f:
                w = _csv.writer(f)
                w.writerow([
                    'tiempo_s', 'x', 'y', 'yaw_deg',
                    'wp_idx', 'wp_x', 'wp_y',
                    'dist_m', 'hdg_err_deg',
                    'cmd_v', 'cmd_w', 'fase',
                ])
                w.writerows(self._log_rows)
            self.get_logger().info(f'CSV → {self._csv_path}')
        except Exception as e:
            self.get_logger().error(f'Error CSV: {e}')

        # JSON
        errors = [
            ev['pos_error_cm'] for ev in self._wp_events
            if ev.get('pos_error_cm') is not None
        ]
        summary = {
            'fecha':           datetime.now().isoformat(),
            'nodo':            self.get_name(),
            'tiempo_total_s':  round(total_time, 2),
            'waypoint_events': self._wp_events,
            'incidentes':      self._incidents,
            'estadisticas': {
                'wps_alcanzados': sum(1 for ev in self._wp_events if ev['resultado'] == 'ALCANZADO'),
                'wps_timeout':    sum(1 for ev in self._wp_events if ev['resultado'] == 'TIMEOUT'),
                'error_prom_cm':  round(sum(errors) / len(errors), 2) if errors else None,
                'error_max_cm':   round(max(errors), 2) if errors else None,
            },
            'diagnostico':     self._diagnose(errors),
            'parametros': {
                'v_max': self._v_max, 'w_max': self._w_max,
                'k_v':   self._k_v,   'k_w':   self._k_w,
                'v_rot': self._v_rot, 'w_min': self._w_min,
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
            self.get_logger().warn(f'Diagnóstico → {self._json_path}')
        except Exception as e:
            self.get_logger().error(f'Error JSON: {e}')

        # Resumen en consola
        self.get_logger().warn('─' * 50)
        for ev in self._wp_events:
            err = f"{ev['pos_error_cm']:.1f}cm" if ev.get('pos_error_cm') is not None else '—'
            self.get_logger().warn(
                f"  WP{ev['waypoint']}: {ev['resultado']}  "
                f"error={err}  tiempo={ev['tiempo_s']:.1f}s"
            )
        for d in self._diagnose(errors):
            self.get_logger().warn(f'  → {d}')
        self.get_logger().warn('─' * 50)

    def _diagnose(self, errors: list[float]) -> list[str]:
        diag = []
        timeouts = [i for i in self._incidents if 'TIMEOUT' in i]
        stales   = [i for i in self._incidents if 'STALE'   in i]
        grows    = [i for i in self._incidents if 'creciendo' in i]

        if stales:
            diag.append(
                'Robot perdió visibilidad del sensor (choque o salida del área). '
                'Reducir v_max o wp_side.'
            )
        if grows:
            diag.append(
                'Robot se alejó del waypoint. '
                'Verificar que /cmd_vel llega al hardware y que los ejes X/Y coinciden.'
            )
        if timeouts:
            diag.append(
                f'{len(timeouts)} WP(s) por timeout. '
                'Aumentar wp_timeout o reducir wp_side.'
            )
        if errors:
            avg = sum(errors) / len(errors)
            if avg > 15:
                diag.append(
                    f'Error alto ({avg:.1f}cm promedio). '
                    'Reducir v_max para un frenado más suave.'
                )
            elif avg < 8:
                diag.append(f'Buena precisión ({avg:.1f}cm promedio) ✓')
            else:
                diag.append(f'Precisión aceptable ({avg:.1f}cm promedio).')
        if not diag:
            diag.append('Sin incidentes registrados.')
        return diag

    def _shutdown(self) -> None:
        """Llamar en finally del main para frenar y guardar log parcial."""
        try:
            self._pub_vel.publish(Twist())
        except Exception:
            pass
        if not self._done and self._log_rows and self._t0:
            self.get_logger().warn('Guardando log parcial...')
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._t0
            self._save_logs(elapsed)
        self.get_logger().info('Robot detenido.')
