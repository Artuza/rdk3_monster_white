"""
Receptor NatNet para OptiTrack — nodo ROS2 en Python puro (sin libNatNet.so).
Soporta modo unicast (conecta directo a Motive) y multicast.

Publica:
  /optitrack/rigid_body  → geometry_msgs/PoseStamped
  /optitrack/pose        → geometry_msgs/PoseStamped  (alias)

Parámetros ROS2:
  server_ip    : IP del PC con Motive       (default: 192.168.0.107)
  local_ip     : IP local de esta máquina   (default: auto-detecta)
  multicast_ip : Grupo multicast            (default: 239.255.42.99)
  command_port : Puerto de comandos NatNet  (default: 1510)
  data_port    : Puerto de datos NatNet     (default: 1511)
  use_multicast: True=multicast, False=unicast (default: False)
  rigid_id     : ID del rigid body, -1=primero (default: -1)
  frame_id     : frame_id del PoseStamped   (default: optitrack)

Modo recomendado (unicast):
  ros2 run monster_trayectoria optitrack_receiver \
       --ros-args -p server_ip:=192.168.0.107 -p local_ip:=192.168.0.141

Modo multicast (si Motive está en multicast):
  ros2 run monster_trayectoria optitrack_receiver \
       --ros-args -p server_ip:=192.168.0.107 -p use_multicast:=true \
                  -p local_ip:=192.168.0.141
"""

import socket
import struct
import threading
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

# ── Constantes NatNet ──────────────────────────────────────────────────────────
NAT_CONNECT      = 0
NAT_SERVERINFO   = 1
NAT_FRAMEOFDATA  = 7
NAT_KEEPALIVE    = 10

COMMAND_PORT_DEFAULT = 1510
DATA_PORT_DEFAULT    = 1511
MULTICAST_IP_DEFAULT = "239.255.42.99"


# ── Parser de frames NatNet v3/v4 ─────────────────────────────────────────────
class NatNetParser:
    def parse_frame(self, data: bytes) -> Optional[list]:
        """
        Parsea un paquete NAT_FRAMEOFDATA (mensaje ID=7).
        Devuelve lista de dicts: {id, x, y, z, qx, qy, qz, qw, valid}
        """
        offset = 0
        try:
            msg_id = struct.unpack_from('<H', data, offset)[0]; offset += 2
            _pkt_sz = struct.unpack_from('<H', data, offset)[0]; offset += 2

            if msg_id != NAT_FRAMEOFDATA:
                return None

            # Frame number
            _frame = struct.unpack_from('<i', data, offset)[0]; offset += 4

            # ── Labeled Marker Sets ───────────────────────────────────────────
            n_sets = struct.unpack_from('<i', data, offset)[0]; offset += 4
            for _ in range(n_sets):
                end = data.index(b'\x00', offset)
                offset = end + 1
                n_markers = struct.unpack_from('<i', data, offset)[0]; offset += 4
                offset += n_markers * 12  # 3 × float32

            # ── Unlabeled Markers ─────────────────────────────────────────────
            n_unlabeled = struct.unpack_from('<i', data, offset)[0]; offset += 4
            offset += n_unlabeled * 12

            # ── Rigid Bodies ──────────────────────────────────────────────────
            n_bodies = struct.unpack_from('<i', data, offset)[0]; offset += 4

            bodies = []
            for _ in range(n_bodies):
                rb_id = struct.unpack_from('<i', data, offset)[0]; offset += 4
                x, y, z = struct.unpack_from('<3f', data, offset); offset += 12
                qx, qy, qz, qw = struct.unpack_from('<4f', data, offset); offset += 16
                _err = struct.unpack_from('<f', data, offset)[0]; offset += 4
                params = struct.unpack_from('<H', data, offset)[0]; offset += 2
                bodies.append({
                    'id': rb_id,
                    'x': x, 'y': y, 'z': z,
                    'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
                    'valid': bool(params & 0x01),
                })
            return bodies

        except Exception:
            return None


# ── Nodo ROS2 ─────────────────────────────────────────────────────────────────
class OptitrackReceiver(Node):
    def __init__(self):
        super().__init__('optitrack_receiver')

        self.declare_parameter('server_ip',    '192.168.0.107')
        self.declare_parameter('local_ip',     '')             # vacío = auto
        self.declare_parameter('multicast_ip', MULTICAST_IP_DEFAULT)
        self.declare_parameter('command_port', COMMAND_PORT_DEFAULT)
        self.declare_parameter('data_port',    DATA_PORT_DEFAULT)
        self.declare_parameter('use_multicast', False)
        self.declare_parameter('rigid_id',     -1)
        self.declare_parameter('frame_id',     'optitrack')

        self._server_ip    = self.get_parameter('server_ip').value
        self._local_ip     = self.get_parameter('local_ip').value
        self._multicast_ip = self.get_parameter('multicast_ip').value
        self._cmd_port     = int(self.get_parameter('command_port').value)
        self._data_port    = int(self.get_parameter('data_port').value)
        self._multicast    = bool(self.get_parameter('use_multicast').value)
        self._rigid_id     = int(self.get_parameter('rigid_id').value)
        self._frame_id     = self.get_parameter('frame_id').value

        if not self._local_ip:
            self._local_ip = self._detect_local_ip()

        self._pub_rigid = self.create_publisher(PoseStamped, '/optitrack/rigid_body', 10)
        self._pub_pose  = self.create_publisher(PoseStamped, '/optitrack/pose', 10)
        self._parser    = NatNetParser()
        self._running   = True

        mode = "multicast" if self._multicast else "unicast"
        self.get_logger().info(
            f'OptiTrack receiver: server={self._server_ip}  local={self._local_ip}'
            f'  mode={mode}  data_port={self._data_port}'
            f'  rigid_id={"auto" if self._rigid_id < 0 else self._rigid_id}'
        )

        self._rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._rx_thread.start()

        # Keepalive para unicast (cada 2s)
        if not self._multicast:
            self._cmd_sock: Optional[socket.socket] = None
            self._ka_timer = self.create_timer(2.0, self._send_keepalive)

    # ── Auto-detectar IP local con ruta hacia el servidor ────────────────────
    def _detect_local_ip(self) -> str:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect((self._server_ip, 1))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return '0.0.0.0'

    # ── Crear socket de datos ─────────────────────────────────────────────────
    def _make_data_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass
        sock.settimeout(1.0)

        if self._multicast:
            sock.bind(('', self._data_port))
            mreq = socket.inet_aton(self._multicast_ip) + socket.inet_aton(self._local_ip)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            self.get_logger().info(f'Unido a multicast {self._multicast_ip}:{self._data_port}')
        else:
            # Unicast: bind al puerto de datos en nuestra IP
            sock.bind((self._local_ip, self._data_port))
            self.get_logger().info(f'Escuchando unicast en {self._local_ip}:{self._data_port}')
            # Conectar al servidor y enviar solicitud de conexión
            self._cmd_sock = self._connect_to_server()

        return sock

    # ── Conectar al servidor NatNet (unicast) ─────────────────────────────────
    def _connect_to_server(self) -> Optional[socket.socket]:
        try:
            cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            cmd.settimeout(2.0)
            cmd.bind((self._local_ip, 0))

            # Paquete Connect: ID=0, size=0
            pkt = struct.pack('<HH', NAT_CONNECT, 0)
            cmd.sendto(pkt, (self._server_ip, self._cmd_port))
            self.get_logger().info(f'Enviada solicitud de conexión a {self._server_ip}:{self._cmd_port}')

            try:
                resp, _ = cmd.recvfrom(4096)
                msg_id = struct.unpack_from('<H', resp, 0)[0]
                if msg_id == NAT_SERVERINFO:
                    self.get_logger().info('Servidor NatNet respondió OK')
            except socket.timeout:
                self.get_logger().warn('Sin respuesta del servidor — continuando de todas formas')

            return cmd
        except Exception as e:
            self.get_logger().error(f'Error conectando al servidor: {e}')
            return None

    # ── Keepalive ─────────────────────────────────────────────────────────────
    def _send_keepalive(self):
        if self._cmd_sock:
            try:
                pkt = struct.pack('<HH', NAT_KEEPALIVE, 0)
                self._cmd_sock.sendto(pkt, (self._server_ip, self._cmd_port))
            except Exception:
                pass

    # ── Loop de recepción ─────────────────────────────────────────────────────
    def _receive_loop(self):
        try:
            data_sock = self._make_data_socket()
        except OSError as e:
            self.get_logger().error(f'Error abriendo socket de datos: {e}')
            return

        first = True
        while self._running and rclpy.ok():
            try:
                data, addr = data_sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(data) < 4:
                continue
            msg_id = struct.unpack_from('<H', data, 0)[0]
            if msg_id != NAT_FRAMEOFDATA:
                continue

            if first:
                first = False
                self.get_logger().info(f'Recibiendo frames NatNet de {addr[0]}')

            bodies = self._parser.parse_frame(data)
            if not bodies:
                continue

            target = None
            if self._rigid_id < 0:
                target = bodies[0]
            else:
                for b in bodies:
                    if b['id'] == self._rigid_id:
                        target = b
                        break

            if target is None:
                continue

            msg = PoseStamped()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            msg.pose.position.x    = float(target['x'])
            msg.pose.position.y    = float(target['y'])
            msg.pose.position.z    = float(target['z'])
            msg.pose.orientation.x = float(target['qx'])
            msg.pose.orientation.y = float(target['qy'])
            msg.pose.orientation.z = float(target['qz'])
            msg.pose.orientation.w = float(target['qw'])

            self._pub_rigid.publish(msg)
            self._pub_pose.publish(msg)

        data_sock.close()

    def destroy_node(self):
        self._running = False
        if hasattr(self, '_cmd_sock') and self._cmd_sock:
            self._cmd_sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OptitrackReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
