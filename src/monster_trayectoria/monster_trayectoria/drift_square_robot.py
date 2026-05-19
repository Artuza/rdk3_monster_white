"""
Open-loop drift-square controller for the physical robot.

Uses timed drive/spin phases instead of odometry since the robot's
odometry topic does not publish real-time data.

Drift corner: at each 90° turn the robot simultaneously applies
  linear.x (forward), linear.y (lateral slide outward) and angular.z
  so it swings around the corner like a car drift instead of stopping
  and spinning in place.
"""

import csv
import math
import os
import yaml
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


# ── Filtro de Kalman para pose 2D [x, y, yaw] ────────────────────────────────
class KalmanPose:
    """
    EKF linealizado para pose 2D.
    Predict: modelo cinemático unicycle con cmd_vel como entrada.
    Update:  medición directa de /odom (pose filtrada por EKF del robot).
    Q y R se cargan de calibration.yaml si existe, sino usa defaults.
    """

    def __init__(self, q_pos: float = 0.01, q_yaw: float = 0.01,
                 r_pos: float = 0.005, r_yaw: float = 0.005) -> None:
        self.x = np.zeros(3)          # estado: [x, y, yaw]
        self.P = np.eye(3) * 0.1      # covarianza inicial

        # Ruido de proceso Q (calibrado con datos reales si disponible)
        self.Q = np.diag([q_pos, q_pos, q_yaw])

        # Ruido de medición R (covarianza del sensor /odom)
        self.R = np.diag([r_pos, r_pos, r_yaw])

        # Factores de corrección de sesgo (calibración empírica)
        self.dist_factor  = 1.0   # real/comandado en distancia
        self.angle_factor = 1.0   # real/comandado en ángulo

    @classmethod
    def from_calibration(cls, yaml_path: str) -> 'KalmanPose':
        """Carga Q, R y factores de corrección desde calibration.yaml."""
        kf = cls()
        if not os.path.exists(yaml_path):
            return kf
        try:
            with open(yaml_path) as f:
                data = yaml.safe_load(f) or {}
            if 'distance' in data:
                q = data['distance'].get('q_noise', 0.01)
                kf.Q[0, 0] = kf.Q[1, 1] = q
                kf.dist_factor = data['distance'].get('correction_factor', 1.0)
            if 'angle' in data:
                q = data['angle'].get('q_noise', 0.01)
                kf.Q[2, 2] = math.radians(q) ** 2   # convertir a rad²
                kf.angle_factor = data['angle'].get('correction_factor', 1.0)
        except Exception:
            pass
        return kf

    def predict(self, vx: float, vy: float, wz: float, dt: float) -> None:
        """Predicción con modelo cinemático (aplicando corrección de sesgo)."""
        vx_corr = vx * self.dist_factor
        vy_corr = vy * self.dist_factor
        wz_corr = wz * self.angle_factor
        yaw = self.x[2]
        # Actualizar estado (mecanum: vx adelante, vy lateral en frame del robot)
        self.x[0] += (vx_corr * math.cos(yaw) - vy_corr * math.sin(yaw)) * dt
        self.x[1] += (vx_corr * math.sin(yaw) + vy_corr * math.cos(yaw)) * dt
        self.x[2] += wz_corr * dt
        self.x[2]  = math.atan2(math.sin(self.x[2]), math.cos(self.x[2]))
        # Jacobiano F (linealización)
        F = np.eye(3)
        F[0, 2] = (-vx_corr * math.sin(yaw) - vy_corr * math.cos(yaw)) * dt
        F[1, 2] = ( vx_corr * math.cos(yaw) - vy_corr * math.sin(yaw)) * dt
        # Propagar covarianza
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z_x: float, z_y: float, z_yaw: float) -> None:
        """Corrección con medición de /odom (H = I, observación directa)."""
        z   = np.array([z_x, z_y, z_yaw])
        inn = z - self.x                          # innovación
        inn[2] = math.atan2(math.sin(inn[2]), math.cos(inn[2]))  # wrap yaw
        S = self.P + self.R                       # H=I → S = P + R
        K = self.P @ np.linalg.inv(S)            # ganancia de Kalman
        self.x = self.x + K @ inn
        self.x[2] = math.atan2(math.sin(self.x[2]), math.cos(self.x[2]))
        self.P = (np.eye(3) - K) @ self.P

    # ── Propiedades de conveniencia ───────────────────────────────────────────
    @property
    def pos_x(self) -> float:   return float(self.x[0])
    @property
    def pos_y(self) -> float:   return float(self.x[1])
    @property
    def yaw(self)   -> float:   return float(self.x[2])
    @property
    def pos_std(self) -> float:
        return float(math.sqrt(self.P[0, 0] + self.P[1, 1]))
    @property
    def yaw_std(self) -> float:
        return float(math.sqrt(abs(self.P[2, 2])))

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    _PLOT_AVAILABLE = True
except ImportError:
    _PLOT_AVAILABLE = False

_CMD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# QoS para sensores que publican BEST_EFFORT (OptiTrack, vel_raw, IMU)
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Expected corner positions for a square starting at origin heading East (+X)
_SQUARE_CORNERS = [
    (0.0, 0.0,  0.0),   # start
    (1.0, 0.0,  90.0),  # after leg 0
    (1.0, 1.0, 180.0),  # after leg 1
    (0.0, 1.0, 270.0),  # after leg 2
    (0.0, 0.0,   0.0),  # back to start after leg 3
]


class DriftSquareRobot(Node):
    def __init__(self) -> None:
        super().__init__('drift_square_robot')

        self.declare_parameter('side',      1.0)
        self.declare_parameter('legs',      4)
        self.declare_parameter('loop',      True)
        self.declare_parameter('v_max',     0.3)
        self.declare_parameter('w_max',     1.0)   # igual que omni_test — valor verificado en robot real
        self.declare_parameter('stop_time',       1.0)   # debe ser suficiente para frenar desde v_max
        self.declare_parameter('stop_after_spin', 1.0)
        self.declare_parameter('spin_time', 0.0)        # 0 = auto: (pi/2)/w_max * spin_factor
        self.declare_parameter('spin_factor', 1.58)    # margen extra sobre el tiempo teórico (medido: 1.4→79.9°)
        self.declare_parameter('drive_correction', 0.0)
        # Drift corner — desactivado por defecto hasta que los giros funcionen limpiamente.
        # Activar con: -p drift_vy:=0.15
        self.declare_parameter('drift_vx', 0.0)
        self.declare_parameter('drift_vy', 0.0)   # 0 = giro puro, sin lateral
        # Warmup: wiggle suave al inicio para cebar los motores antes del primer giro
        self.declare_parameter('warmup_time', 0.15)  # segundos por dirección (izq+der)
        self.declare_parameter('warmup_stop', 0.5)   # pausa tras el wiggle
        # Control de heading durante DRIVE (P controller con IMU)
        self.declare_parameter('heading_kp',   1.0)   # corrección de heading (yaw) durante DRIVE
        self.declare_parameter('lateral_kp',  0.3)   # corrección lateral feedback (slip mecanum)
        self.declare_parameter('lateral_bias', 0.12)  # offset feedforward (m/s) corrección de bias mecánico
        # Filtro de rigid body por nombre (vacío = acepta cualquiera)
        self.declare_parameter('rigid_name', '')
        # Timeout de DRIVE (0 = auto: max(drive_time*6, 20.0))
        self.declare_parameter('drive_timeout', 0.0)
        # Controlador P para giro en DRIFT
        self.declare_parameter('spin_kp',    2.5)   # rad/s por rad de error
        self.declare_parameter('spin_w_min', 0.3)   # velocidad mínima para vencer fricción

        self._side       = float(self.get_parameter('side').value)
        self._num_legs   = int(self.get_parameter('legs').value)
        self._loop       = self.get_parameter('loop').value
        self._v_max      = self.get_parameter('v_max').value
        self._w_max      = self.get_parameter('w_max').value
        self._stop_time       = self.get_parameter('stop_time').value
        self._stop_after_spin = self.get_parameter('stop_after_spin').value
        _spin_time_param      = self.get_parameter('spin_time').value
        self._drive_correction = self.get_parameter('drive_correction').value
        self._drift_vx = self.get_parameter('drift_vx').value
        self._drift_vy = self.get_parameter('drift_vy').value
        self._spin_factor = self.get_parameter('spin_factor').value
        self._warmup_time  = self.get_parameter('warmup_time').value
        self._warmup_stop  = self.get_parameter('warmup_stop').value
        self._heading_kp    = float(self.get_parameter('heading_kp').value)
        self._lateral_kp    = float(self.get_parameter('lateral_kp').value)
        self._lateral_bias  = float(self.get_parameter('lateral_bias').value)
        self._rigid_name   = self.get_parameter('rigid_name').value.strip()
        _drive_timeout_param = float(self.get_parameter('drive_timeout').value)
        self._spin_kp    = float(self.get_parameter('spin_kp').value)
        self._spin_w_min = float(self.get_parameter('spin_w_min').value)

        # Time needed to cover one side and drift 90°
        self._drive_time = self._side / self._v_max
        _auto_spin = (math.pi / 2) / self._w_max * self._spin_factor
        self._spin_time  = _spin_time_param if _spin_time_param > 0.0 else _auto_spin

        self._pub_vel = self.create_publisher(Twist, '/cmd_vel', _CMD_QOS)

        # ── Filtro de Kalman ──────────────────────────────────────────────────
        calib_path = os.path.expanduser('~/ros2_ws/calibration.yaml')
        self._kf = KalmanPose.from_calibration(calib_path)
        self._kf_last_t  = -1.0          # timestamp del último predict
        self._odom_valid = False          # primer mensaje de /odom recibido
        self._kf_start_x   = 0.0         # pose KF al inicio de cada fase
        self._kf_start_y   = 0.0
        self._kf_start_yaw = 0.0
        if os.path.exists(calib_path):
            self.get_logger().info(
                f'Kalman cargado desde {calib_path}  '
                f'dist_factor={self._kf.dist_factor:.3f}  '
                f'angle_factor={self._kf.angle_factor:.3f}'
            )
        else:
            self.get_logger().info('Kalman con parámetros por defecto (sin calibration.yaml)')

        # Subscribers para datos reales
        self.create_subscription(Imu,        '/imu/data_raw',          self._cb_imu,        10)
        self.create_subscription(Odometry,   '/odom',                  self._cb_odom,       10)
        self.create_subscription(JointState, '/joint_states',          self._cb_joints,     10)
        self.create_subscription(Float32,    '/voltage',               self._cb_voltage,    10)
        self.create_subscription(Twist,      '/vel_raw',               self._cb_vel_raw,    10)
        self.create_subscription(PoseStamped, '/optitrack/rigid_body', self._cb_optitrack,
                                 _SENSOR_QOS)

        # Estado OptiTrack
        self._optitrack_valid   = False   # primer mensaje recibido
        self._optitrack_origin  = None    # (x, y, yaw) en el frame de OptiTrack al inicio
        self._opt_x   = 0.0              # pose relativa actual de OptiTrack (para CSV)
        self._opt_y   = 0.0
        self._opt_yaw = 0.0
        self._opt_yaw_drift_start = 0.0  # opt_yaw al inicio de cada DRIFT
        self._opt_last_kf_x = 0.0        # última posición enviada al KF (evitar updates repetidos)
        self._opt_last_kf_y = 0.0
        # R para OptiTrack — más conservador para evitar saltos por mensajes atrasados
        self._kf_r_optitrack = np.diag([0.003, 0.003, 0.005])
        # Última posición de odom enviada al KF (evitar updates cuando odom está congelado)
        self._odom_last_x = None
        self._odom_last_y = None

        # Buffers datos reales
        self._real_t_imu    = []; self._real_wz      = []; self._real_ax = []
        self._real_t_odom   = []; self._real_odom_x  = []; self._real_odom_y = []
        self._real_t_joint  = []; self._real_wl      = []; self._real_wr = []
        self._real_t_volt   = []; self._real_voltage  = []

        # vel_raw: velocidad real medida por el MCU
        self._raw_vx = 0.0
        self._raw_vy = 0.0
        self._raw_wz = 0.0

        # Control cerrado por sensores
        self._enc_dist   = 0.0   # distancia integrada desde encoders (DRIVE)
        self._enc_last_t = -1.0  # timestamp del último vel_raw para calcular dt
        self._imu_yaw    = 0.0   # yaw integrado desde giroscopio IMU (DRIFT, se resetea)
        self._imu_last_t = -1.0  # timestamp del último IMU para calcular dt
        # Heading continuo (no se resetea) — para corrección de rumbo en DRIVE
        self._imu_heading         = 0.0   # integral continua de wz del IMU
        self._imu_heading_t       = -1.0
        self._drive_start_heading = 0.0   # heading al inicio del tramo actual
        # Comando previo — para KF predict correcto
        self._prev_cmd_vx = 0.0
        self._prev_cmd_vy = 0.0
        self._prev_cmd_wz = 0.0

        # drive_timeout >> drive_time para que el stop lo dispare kf_dist/encoder, no el tiempo
        self._drive_timeout = (_drive_timeout_param if _drive_timeout_param > 0.0
                               else max(self._drive_time * 6, 20.0))
        self._spin_timeout  = self._spin_time

        # CSV log incremental
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir = os.path.expanduser('~/ros2_ws/logs')
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, f'drift_log_{stamp}.csv')
        self._csv_file = open(log_path, 'w', newline='')
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow([
            't_s', 'leg', 'state', 'phase_elapsed_s',
            'cmd_vx', 'cmd_vy', 'cmd_wz',
            'raw_vx', 'raw_vy', 'raw_wz',
            'enc_dist_m', 'imu_yaw_deg', 'imu_heading_deg',
            'kf_x', 'kf_y', 'kf_yaw_deg',
            'opt_x', 'opt_y', 'opt_yaw_deg',
            'lateral_err_m', 'heading_err_deg',
            'event',
        ])
        self._csv_file.flush()
        self.get_logger().info(f'CSV log: {log_path}')

        self._state       = 'WARMUP'
        self._leg         = 0
        self._phase_start = self.get_clock().now()
        self._done        = False
        self._log_tick    = 0

        # Estimated pose (open-loop dead reckoning)
        self._x     = 0.0
        self._y     = 0.0
        self._yaw   = 0.0   # radians, 0 = East (+X)

        # Data recording for plots
        self._t0          = self.get_clock().now()
        self._rec_t       = []
        self._rec_linx    = []
        self._rec_liny    = []
        self._rec_angz    = []
        self._rec_x       = []
        self._rec_y       = []
        self._rec_state   = []
        _STATE_ID = {'WARMUP': -1, 'DRIVE': 0, 'STOP_BEFORE_SPIN': 1, 'DRIFT': 2, 'STOP_BEFORE_DRIVE': 3}
        self._STATE_ID    = _STATE_ID

        # Phase duration log: list of (phase_name, duration_s, leg_index)
        self._phase_log       = []
        self._phase_log_start = self._t0

        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f'Drift-square ROBOT ready  side={self._side:.1f} m  '
            f'drive={self._drive_time:.1f}s  stop={self._stop_time:.1f}s  '
            f'drift={self._spin_time:.1f}s  loop={self._loop}'
        )
        self.get_logger().info(
            f'Drift params  vx={self._drift_vx:.2f} m/s  '
            f'vy={self._drift_vy:.2f} m/s (lateral)  wz={self._w_max:.2f} rad/s  '
            f'spin_time={self._spin_time:.2f}s (factor={self._spin_factor:.1f})'
        )
        self.get_logger().info(
            f'Posicion inicial  x={self._x:.2f} m  y={self._y:.2f} m  '
            f'yaw={math.degrees(self._yaw):.1f}°'
        )

    def _now_s(self) -> float:
        return (self.get_clock().now() - self._t0).nanoseconds * 1e-9

    def _cb_imu(self, msg: Imu) -> None:
        now = self._now_s()
        wz  = msg.angular_velocity.z

        # Heading continuo (siempre integra) — para corrección de rumbo en DRIVE
        if self._imu_heading_t >= 0.0:
            dt = now - self._imu_heading_t
            if 0.001 < dt < 0.2:
                self._imu_heading += wz * dt
        self._imu_heading_t = now

        # Integrar yaw durante DRIFT para control cerrado (se resetea al inicio de DRIFT)
        if self._state == 'DRIFT' and self._imu_last_t >= 0.0:
            dt = now - self._imu_last_t
            if 0.001 < dt < 0.2:
                self._imu_yaw += wz * dt
        self._imu_last_t = now

        self._real_t_imu.append(now)
        self._real_wz.append(wz)
        self._real_ax.append(msg.linear_acceleration.x)

    def _cb_odom(self, msg: Odometry) -> None:
        now = self._now_s()
        ox  = msg.pose.pose.position.x
        oy  = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom_valid = True
        # Solo actualizar KF si el odom realmente se ha movido (evita corromper
        # dead reckoning cuando el odom está congelado en el origen)
        if self._odom_last_x is not None:
            if math.hypot(ox - self._odom_last_x, oy - self._odom_last_y) < 0.002:
                self._real_t_odom.append(now)
                self._real_odom_x.append(ox)
                self._real_odom_y.append(oy)
                return
        self._odom_last_x = ox
        self._odom_last_y = oy
        self._kf.update(ox, oy, yaw)
        self._real_t_odom.append(now)
        self._real_odom_x.append(ox)
        self._real_odom_y.append(oy)

    def _cb_joints(self, msg: JointState) -> None:
        if len(msg.velocity) < 2:
            return
        self._real_t_joint.append(self._now_s())
        self._real_wl.append(msg.velocity[0])
        self._real_wr.append(msg.velocity[1])

    def _cb_voltage(self, msg: Float32) -> None:
        self._real_t_volt.append(self._now_s())
        self._real_voltage.append(msg.data)

    def _cb_vel_raw(self, msg: Twist) -> None:
        now = self._now_s()
        self._raw_vx = msg.linear.x
        self._raw_vy = msg.linear.y
        self._raw_wz = msg.angular.z
        # Integrar distancia durante DRIVE para control cerrado
        if self._state == 'DRIVE' and self._enc_last_t >= 0.0:
            dt = now - self._enc_last_t
            if 0.001 < dt < 0.2:
                self._enc_dist += max(0.0, self._raw_vx) * dt
        self._enc_last_t = now

    def _cb_optitrack(self, msg: PoseStamped) -> None:
        # Filtrar por nombre de rigid body si se configuró
        if self._rigid_name and msg.header.frame_id != self._rigid_name:
            return
        ox = msg.pose.position.x
        oy = msg.pose.position.y
        q  = msg.pose.orientation
        ot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        if not self._optitrack_valid:
            self._optitrack_origin = (ox, oy, ot_yaw)
            self._optitrack_valid  = True
            self.get_logger().info(
                f'OptiTrack origen fijado  frame_id={msg.header.frame_id}'
                f'  x={ox:.3f} y={oy:.3f} yaw={math.degrees(ot_yaw):.1f}°'
            )
            return

        ox0, oy0, oyaw0 = self._optitrack_origin
        dx = ox - ox0
        dy = oy - oy0
        dyaw = math.atan2(math.sin(ot_yaw - oyaw0), math.cos(ot_yaw - oyaw0))
        rel_x =  dx * math.cos(-oyaw0) - dy * math.sin(-oyaw0)
        rel_y =  dx * math.sin(-oyaw0) + dy * math.cos(-oyaw0)

        # Guardar para CSV
        self._opt_x   = rel_x
        self._opt_y   = rel_y
        self._opt_yaw = dyaw

        # Solo actualizar KF si OptiTrack reporta un movimiento real (>5mm)
        # — evita congelar el dead reckoning del KF cuando se pierde el tracking
        if math.hypot(rel_x - self._opt_last_kf_x,
                      rel_y - self._opt_last_kf_y) >= 0.005:
            self._opt_last_kf_x = rel_x
            self._opt_last_kf_y = rel_y
            r_prev = self._kf.R.copy()
            self._kf.R = self._kf_r_optitrack
            self._kf.update(rel_x, rel_y, dyaw)
            self._kf.R = r_prev

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._phase_start).nanoseconds * 1e-9

    def _next_phase(self, new_state: str) -> None:
        now = self.get_clock().now()
        dur = (now - self._phase_log_start).nanoseconds * 1e-9
        self._phase_log.append((self._state, dur, self._leg))
        self._phase_log_start = now
        self._state = new_state
        self._phase_start = now
        # Resetear integradores — siempre limpiar dt para evitar saltos en nueva fase
        self._imu_last_t = -1.0
        self._enc_last_t = -1.0
        self._kf_last_t  = -1.0
        if new_state == 'DRIVE':
            self._enc_dist = 0.0
            self._kf_start_x   = self._kf.pos_x
            self._kf_start_y   = self._kf.pos_y
            self._kf_start_yaw = self._kf.yaw
            self._drive_start_heading = self._imu_heading  # heading IMU al inicio del tramo
            # Resetear referencia de OptiTrack para evitar que una posición
            # congelada del tramo anterior bloquee las actualizaciones del KF
            self._opt_last_kf_x = self._opt_x
            self._opt_last_kf_y = self._opt_y
        elif new_state == 'DRIFT':
            self._imu_yaw = 0.0
            # Capturar yaw de referencia al inicio del giro
            self._kf_start_yaw = self._kf.yaw
            self._opt_yaw_drift_start = self._opt_yaw  # OptiTrack directo, sin dead reckoning

    def _lateral_error(self) -> float:
        """Desviación lateral (m) respecto al rumbo inicial del tramo. + = izquierda."""
        dx = self._kf.pos_x - self._kf_start_x
        dy = self._kf.pos_y - self._kf_start_y
        heading = self._kf_start_yaw
        return -dx * math.sin(heading) + dy * math.cos(heading)

    def _csv_row(self, cmd: Twist, event: str = '') -> None:
        t = self._now_s()
        if self._optitrack_valid:
            imu_hdg_err = math.atan2(
                math.sin(self._kf.yaw - self._kf_start_yaw),
                math.cos(self._kf.yaw - self._kf_start_yaw))
        else:
            imu_hdg_err = math.atan2(
                math.sin(self._imu_heading - self._drive_start_heading),
                math.cos(self._imu_heading - self._drive_start_heading))
        self._csv.writerow([
            f'{t:.3f}', self._leg, self._state, f'{self._elapsed():.3f}',
            f'{cmd.linear.x:.3f}', f'{cmd.linear.y:.3f}', f'{cmd.angular.z:.3f}',
            f'{self._raw_vx:.3f}', f'{self._raw_vy:.3f}', f'{self._raw_wz:.3f}',
            f'{self._enc_dist:.3f}', f'{math.degrees(self._imu_yaw):.1f}',
            f'{math.degrees(self._imu_heading):.1f}',
            f'{self._kf.pos_x:.4f}', f'{self._kf.pos_y:.4f}',
            f'{math.degrees(self._kf.yaw):.2f}',
            f'{self._opt_x:.4f}', f'{self._opt_y:.4f}',
            f'{math.degrees(self._opt_yaw):.2f}',
            f'{self._lateral_error():.4f}',
            f'{math.degrees(imu_hdg_err):.2f}',
            event,
        ])
        if event:
            self._csv_file.flush()  # asegurar escritura en eventos importantes

    def _log_position(self, event: str) -> None:
        expected = _SQUARE_CORNERS[self._leg] if self._leg < len(_SQUARE_CORNERS) else None
        msg = (
            f'[{event}]  '
            f'x={self._x:.2f} m  y={self._y:.2f} m  yaw={math.degrees(self._yaw):.1f}°'
        )
        if expected is not None:
            ex, ey, eyaw = expected
            ex = ex * self._side
            ey = ey * self._side
            msg += f'  |  esperado x={ex:.2f} m  y={ey:.2f} m  yaw={eyaw:.1f}°'
        self.get_logger().info(msg)

    def _control_loop(self) -> None:
        cmd = Twist()

        if self._done:
            self._pub_vel.publish(cmd)
            return

        elapsed = self._elapsed()

        # ── Kalman predict con comando del tick anterior ──────────────────────
        now_s = self._now_s()
        if self._kf_last_t >= 0.0:
            kf_dt = now_s - self._kf_last_t
            if 0.001 < kf_dt < 0.2:
                self._kf.predict(self._prev_cmd_vx, self._prev_cmd_vy, self._prev_cmd_wz, kf_dt)
        self._kf_last_t = now_s

        if self._state == 'WARMUP':
            # Wiggle suave (izq→der→pausa) para cebar motores antes del primer giro
            total = 2 * self._warmup_time + self._warmup_stop
            if elapsed < self._warmup_time:
                cmd.angular.z = self._w_max * 0.5
            elif elapsed < 2 * self._warmup_time:
                cmd.angular.z = -self._w_max * 0.5
            elif elapsed >= total:
                self.get_logger().info('Warmup completo — iniciando trayectoria')
                self._csv_row(cmd, 'FIN_WARMUP')
                self._next_phase('DRIVE')
            self._csv_row(cmd)
            self._pub_vel.publish(cmd)
            return

        elif self._state == 'DRIVE':
            # Distancia KF recorrida en este tramo
            kf_dist = math.hypot(self._kf.pos_x - self._kf_start_x,
                                 self._kf.pos_y - self._kf_start_y)
            # enc_dist (vel_raw) es la fuente principal — real, desde encoders del robot.
            # kf_dist (dead reckoning) solo si vel_raw sigue a cero (bridge sin datos).
            if self._enc_dist >= 0.01:
                sensor_done = self._enc_dist >= self._side
            else:
                sensor_done = kf_dist >= self._side
            timeout_done = elapsed >= self._drive_timeout
            if sensor_done or timeout_done:
                if self._enc_dist >= self._side:
                    trigger = f'encoder_{self._enc_dist:.2f}m'
                elif kf_dist >= self._side:
                    trigger = f'KF_{kf_dist:.2f}m'
                else:
                    trigger = f'TIMEOUT_{elapsed:.1f}s'
                self._x += self._side * math.cos(self._yaw)
                self._y += self._side * math.sin(self._yaw)
                self._log_position(f'Fin tramo {self._leg}')
                self._csv_row(cmd, f'FIN_DRIVE_leg{self._leg}_{trigger}')
                self._log_tick = 0
                self._next_phase('STOP_BEFORE_SPIN')
            else:
                cmd.linear.x = self._v_max
                # Corrección de rumbo (yaw) con KF/OptiTrack como fuente principal
                if self._optitrack_valid:
                    heading_err = math.atan2(
                        math.sin(self._kf.yaw - self._kf_start_yaw),
                        math.cos(self._kf.yaw - self._kf_start_yaw))
                else:
                    heading_err = math.atan2(
                        math.sin(self._imu_heading - self._drive_start_heading),
                        math.cos(self._imu_heading - self._drive_start_heading))
                cmd.angular.z = -self._heading_kp * heading_err
                # Corrección de slip lateral (mecanum): cmd.linear.y contrarresta la deriva
                if self._optitrack_valid:
                    lat_err = self._lateral_error()
                    cmd.linear.y = -(self._lateral_kp * lat_err + self._lateral_bias)

                self._log_tick += 1
                if self._log_tick % 5 == 0:  # cada ~0.25 s
                    best = max(self._enc_dist, kf_dist)
                    pct  = min(best / self._side * 100.0, 100.0)
                    bar  = '█' * int(pct / 5) + '░' * (20 - int(pct / 5))
                    src  = 'enc' if self._enc_dist >= kf_dist else 'OPT'
                    lat  = self._lateral_error()
                    self.get_logger().info(
                        f'[DRIVE {self._leg}]  {bar}  {best:.2f}/{self._side:.1f}m ({src})'
                        f'  lat={lat:+.3f}m  hdg_err={math.degrees(heading_err):+.1f}°'
                        f'  σ={self._kf.pos_std:.3f}'
                    )

        elif self._state == 'STOP_BEFORE_SPIN':
            if elapsed >= self._stop_time:
                self.get_logger().info(
                    f'DRIFT esquina {self._leg}  '
                    f'vx={self._drift_vx:.2f}  vy={-self._drift_vy:.2f}  wz={self._w_max:.2f}'
                )
                self._csv_row(cmd, f'INICIO_DRIFT_leg{self._leg}')
                self._next_phase('DRIFT')

        elif self._state == 'DRIFT':
            # Ángulo girado desde el inicio del DRIFT según OptiTrack
            opt_dyaw     = math.atan2(math.sin(self._opt_yaw - self._opt_yaw_drift_start),
                                      math.cos(self._opt_yaw - self._opt_yaw_drift_start))
            opt_dyaw_abs = abs(opt_dyaw)
            imu_abs      = abs(self._imu_yaw)
            # Controlador P: desacelera conforme se acerca a 90°
            # Fuente: OptiTrack si válido, IMU como fallback
            if self._optitrack_valid:
                angle_measured = opt_dyaw_abs
            else:
                angle_measured = imu_abs
            angle_err   = math.pi / 2 - angle_measured   # error restante hasta 90°
            sensor_done  = angle_err <= 0.0               # llegó o pasó 90°
            timeout_done = elapsed >= self._spin_timeout
            if sensor_done or timeout_done:
                if not timeout_done:
                    trigger = (f'OPT_{math.degrees(opt_dyaw_abs):.1f}deg'
                               if self._optitrack_valid
                               else f'IMU_{math.degrees(imu_abs):.1f}deg')
                else:
                    trigger = f'TIMEOUT_{elapsed:.1f}s'
                # Update estimated yaw after 90° turn
                self._yaw += math.pi / 2
                self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))
                self._leg += 1
                if self._optitrack_valid:
                    actual_turn = math.degrees(opt_dyaw_abs)
                    self.get_logger().info(
                        f'Fin giro {self._leg - 1}: trigger={trigger}'
                        f'  opt_dyaw={actual_turn:.1f}°  error={actual_turn - 90.0:+.1f}°'
                    )
                else:
                    self.get_logger().info(
                        f'Fin giro {self._leg - 1}: trigger={trigger}  opt no disponible'
                    )
                self._csv_row(cmd, f'FIN_DRIFT_leg{self._leg - 1}_{trigger}_{elapsed:.2f}s')
                if self._leg >= self._num_legs:
                    if self._loop:
                        self.get_logger().info(
                            f'--- Cuadrado completo  x={self._x:.2f} m  y={self._y:.2f} m  '
                            f'yaw={math.degrees(self._yaw):.1f}°  --- Reiniciando ---'
                        )
                        self._leg = 0
                        self._x = 0.0
                        self._y = 0.0
                        self._yaw = 0.0
                        self._next_phase('STOP_BEFORE_DRIVE')
                    else:
                        self._done = True
                        self.get_logger().info(
                            f'Cuadrado completo.  x={self._x:.2f} m  y={self._y:.2f} m  '
                            f'yaw={math.degrees(self._yaw):.1f}°'
                        )
                        self._pub_vel.publish(cmd)
                        self._csv_file.flush()
                        return
                else:
                    self.get_logger().info(
                        f'Listo para tramo {self._leg}  '
                        f'yaw estimado={math.degrees(self._yaw):.1f}°  '
                        f'(esperado={90.0 * self._leg:.0f}°)'
                    )
                    self._next_phase('STOP_BEFORE_DRIVE')
            else:
                cmd.linear.x  =  self._drift_vx
                cmd.linear.y  = -self._drift_vy
                # Controlador P: w = Kp × error, clamped a [w_min, w_max]
                # Dither monotónico para que cada msg sea diferente (bypass publish_stale_data:false)
                w_p = max(self._spin_w_min, min(self._w_max, self._spin_kp * angle_err))
                cmd.angular.z = w_p + elapsed * 1e-6
                if int(elapsed / 0.25) != int((elapsed - 0.05) / 0.25):
                    pct_d = min(angle_measured / (math.pi / 2) * 100.0, 100.0)
                    bar   = '█' * int(pct_d / 5) + '░' * (20 - int(pct_d / 5))
                    src   = 'OPT' if self._optitrack_valid else 'IMU'
                    self.get_logger().info(
                        f'[DRIFT {self._leg}]  {bar}  {math.degrees(angle_measured):.1f}°/90° ({src})'
                        f'  err={math.degrees(angle_err):+.1f}°  w={w_p:.2f} rad/s'
                        f'  opt_d={math.degrees(opt_dyaw):+.1f}°'
                    )

        elif self._state == 'STOP_BEFORE_DRIVE':
            if elapsed >= self._stop_after_spin:
                self._next_phase('DRIVE')

        # Record data every tick
        t_now = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        dist  = elapsed * self._v_max if self._state == 'DRIVE' else 0.0
        self._rec_t.append(t_now)
        self._rec_linx.append(cmd.linear.x)
        self._rec_liny.append(cmd.linear.y)
        self._rec_angz.append(cmd.angular.z)
        self._rec_x.append(self._x + dist * math.cos(self._yaw))
        self._rec_y.append(self._y + dist * math.sin(self._yaw))
        self._rec_state.append(self._STATE_ID.get(self._state, -1))

        self._prev_cmd_vx = cmd.linear.x
        self._prev_cmd_vy = cmd.linear.y
        self._prev_cmd_wz = cmd.angular.z
        self._csv_row(cmd)
        self._pub_vel.publish(cmd)

    def save_plots(self) -> None:
        if not _PLOT_AVAILABLE or len(self._rec_t) < 2:
            return

        stamp   = datetime.now().strftime('%Y%m%d_%H%M%S')
        outdir  = os.path.expanduser('~/ros2_ws/logs')
        os.makedirs(outdir, exist_ok=True)

        s = self._side
        ideal_x = [0, s, s, 0, 0]
        ideal_y = [0, 0, s, s, 0]
        state_labels = ['DRIVE', 'STOP→DRIFT', 'DRIFT', 'STOP→DRIVE']
        state_colors = ['#2196F3', '#FF9800', '#F44336', '#4CAF50']

        # ── Figura 1: Control (estimado) ──────────────────────────────────
        fig1, axes = plt.subplots(2, 2, figsize=(13, 9))
        fig1.suptitle(f'Control estimado  —  {stamp}', fontsize=13)

        ax = axes[0, 0]
        ax.plot(ideal_x, ideal_y, 'k--', linewidth=1.5)
        ax.scatter(self._rec_x, self._rec_y, c=self._rec_state,
                   cmap='tab10', vmin=0, vmax=3, s=6, zorder=3)
        ax.plot(self._rec_x[0], self._rec_y[0], 'go', ms=8)
        ax.plot(self._rec_x[-1], self._rec_y[-1], 'rs', ms=8)
        patches = [mpatches.Patch(color=state_colors[i], label=state_labels[i]) for i in range(4)]
        ax.legend(handles=patches, fontsize=7)
        ax.set_title('Trayectoria estimada XY')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

        ax = axes[0, 1]
        ax.plot(self._rec_t, self._rec_linx, color='#2196F3', linewidth=1, label='linear.x')
        ax.plot(self._rec_t, self._rec_liny, color='#00BCD4', linewidth=1, label='linear.y (drift)')
        ax.set_title('Velocidades lineales comandadas'); ax.set_xlabel('Tiempo (s)')
        ax.set_ylabel('m/s'); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        ax = axes[1, 0]
        ax.plot(self._rec_t, self._rec_angz, color='#F44336', linewidth=1)
        ax.set_title('Velocidad angular comandada'); ax.set_xlabel('Tiempo (s)')
        ax.set_ylabel('angular.z (rad/s)'); ax.grid(True, alpha=0.3)

        ax = axes[1, 1]
        ax.step(self._rec_t, self._rec_state, where='post', color='#9C27B0', linewidth=1.5)
        ax.set_yticks([0, 1, 2, 3]); ax.set_yticklabels(state_labels, fontsize=8)
        ax.set_title('Fases del controlador'); ax.set_xlabel('Tiempo (s)')
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        f1 = os.path.join(outdir, f'control_{stamp}.png')
        fig1.savefig(f1, dpi=120); plt.close(fig1)

        # ── Figura 2: Análisis real ───────────────────────────────────────
        fig2, axes = plt.subplots(2, 2, figsize=(13, 9))
        fig2.suptitle(f'Análisis real  —  {stamp}', fontsize=13)

        # 1. Odom real vs estimado
        ax = axes[0, 0]
        ax.plot(ideal_x, ideal_y, 'k--', linewidth=1.5, label='Ideal')
        ax.plot(self._rec_x, self._rec_y, color='#2196F3',
                linewidth=0.8, alpha=0.5, label='Estimado')
        if self._real_odom_x:
            ax.plot(self._real_odom_x, self._real_odom_y, color='#E91E63',
                    linewidth=1.5, label='Odom real')
        else:
            ax.text(0.5, 0.5, 'Odom no disponible\n(robot no publica datos)',
                    ha='center', va='center', transform=ax.transAxes,
                    fontsize=9, color='gray')
        ax.legend(fontsize=7); ax.set_aspect('equal')
        ax.set_title('Trayectoria: odom real vs estimada')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.grid(True, alpha=0.3)

        # 2. Velocidad angular comandada + fases superpuestas
        ax = axes[0, 1]
        ax2 = ax.twinx()
        ax.plot(self._rec_t, self._rec_angz, color='#F44336', linewidth=1,
                label='angular.z cmd', zorder=3)
        ax2.step(self._rec_t, self._rec_state, where='post',
                 color='#9C27B0', linewidth=1, alpha=0.4, label='Fase')
        ax2.set_yticks([0, 1, 2, 3])
        ax2.set_yticklabels(state_labels, fontsize=7, color='#9C27B0')
        ax.set_title('Vel. angular + fases del controlador')
        ax.set_xlabel('Tiempo (s)'); ax.set_ylabel('rad/s', color='#F44336')
        ax.grid(True, alpha=0.3)
        lines1, lab1 = ax.get_legend_handles_labels()
        ax.legend(lines1, lab1, fontsize=7, loc='upper left')

        # 3. Duración de cada fase por iteración
        ax = axes[1, 0]
        drive_durs  = [(i, d) for i, (n, d, _) in enumerate(self._phase_log) if n == 'DRIVE']
        spin_durs   = [(i, d) for i, (n, d, _) in enumerate(self._phase_log) if n == 'DRIFT']
        if drive_durs:
            ax.bar([x[0] for x in drive_durs], [x[1] for x in drive_durs],
                   color='#2196F3', alpha=0.7, label=f'DRIVE (target {self._drive_time:.1f}s)')
        if spin_durs:
            ax.bar([x[0] for x in spin_durs], [x[1] for x in spin_durs],
                   color='#F44336', alpha=0.7, label=f'DRIFT (target {self._spin_time:.1f}s)')
        ax.axhline(self._drive_time, color='#2196F3', linestyle='--', linewidth=1)
        ax.axhline(self._spin_time,  color='#F44336', linestyle='--', linewidth=1)
        ax.set_title('Duración real de cada fase')
        ax.set_xlabel('Nro. de fase'); ax.set_ylabel('Duración (s)')
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        # 4. Error de posición estimada vs ideal en cada esquina
        ax = axes[1, 1]
        corners_ideal = [(c[0] * s, c[1] * s) for c in _SQUARE_CORNERS]
        leg_labels, errors = [], []
        vuelta = 0
        for phase, _dur, leg in self._phase_log:
            if phase == 'STOP_BEFORE_SPIN':
                errors.append(0.0)  # dead-reckoning siempre coincide con ideal
                leg_labels.append(f'V{vuelta}T{leg}')
                if leg == 3:
                    vuelta += 1
        # Error real: comparar odom con ideal en los tiempos de fin de tramo
        if self._real_odom_x and len(self._real_odom_x) > 1:
            # Interpolar odom en tiempos de transición STOP_BEFORE_SPIN
            trans_times = [self._rec_t[i] for i in range(1, len(self._rec_state))
                           if self._rec_state[i] == 1 and self._rec_state[i-1] == 0]
            odom_errors = []
            corner_idx  = 1
            for tt in trans_times:
                # encontrar odom más cercano
                closest = min(range(len(self._real_t_odom)),
                              key=lambda i: abs(self._real_t_odom[i] - tt))
                ox = self._real_odom_x[closest]
                oy = self._real_odom_y[closest]
                ci = corner_idx % 5
                ex, ey = corners_ideal[ci]
                odom_errors.append(math.hypot(ox - ex, oy - ey))
                corner_idx += 1
            if odom_errors:
                ax.bar(range(len(odom_errors)), odom_errors,
                       color='#E91E63', alpha=0.8, label='Error odom vs ideal')
                ax.legend(fontsize=7)
        else:
            ax.text(0.5, 0.5, 'Sin datos de odom\npara calcular error real',
                    ha='center', va='center', transform=ax.transAxes,
                    fontsize=9, color='gray')
        ax.set_title('Error posición en cada esquina (odom vs ideal)')
        ax.set_xlabel('Esquina'); ax.set_ylabel('Error (m)'); ax.grid(True, alpha=0.3)

        plt.tight_layout()
        f2 = os.path.join(outdir, f'analisis_{stamp}.png')
        fig2.savefig(f2, dpi=120); plt.close(fig2)

        self.get_logger().info(f'Gráficas guardadas en {outdir}/')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriftSquareRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped.')
    finally:
        import time
        try:
            node._pub_vel.publish(Twist())
            time.sleep(0.15)  # asegura que el mensaje llega antes de cerrar
        except Exception:
            pass
        try:
            node._csv_file.flush()
            node._csv_file.close()
        except Exception:
            pass
        node.save_plots()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

