"""웹 브리지 — HTTP(정적 페이지) + 단일 WebSocket(/ws) 서버. 외부 패키지 없이 표준 라이브러리만 사용.

브라우저 UI 와 노트북 자동주차 클라이언트가 같은 JSON 메시지 규격을 사용한다 (README '메시지 규격' 참고).
  서버→클라이언트: welcome, state(10 Hz), map(갱신 시), scan(5 Hz), reply, pong
  클라이언트→서버: hello, ping, drive, stop, estop, estop_reset, mode, claim, release,
                  set_control_mode, set_limits, trailer_zero, save_map
제어 관련 요청은 그대로 /rc/command_request 로 Command Manager 에 전달 (권한 판단은 Command Manager).
접근 제한: config web.allowed_networks 에 속한 IP 만 허용 (로그인/토큰 없음 — 신뢰 가능한 로컬망 전제).
"""
import asyncio
import base64
import hashlib
import ipaddress
import itertools
import json
import math
import os
import struct
import threading
import time
import zlib

import rclpy
import rclpy.executors
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .common import dumps, finite_or_none, load_config

try:
    from slam_toolbox.srv import SerializePoseGraph
except ImportError:  # slam_toolbox 파이썬 인터페이스가 없으면 posegraph 저장만 비활성화
    SerializePoseGraph = None

WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
FORWARD = {'drive', 'stop', 'estop', 'estop_reset', 'mode', 'claim', 'release',
           'set_control_mode', 'set_limits', 'trailer_zero'}


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ===================================================================== ROS 측
class BridgeNode(Node):
    def __init__(self):
        super().__init__('rc_web_bridge')
        self.declare_parameter('config_file', '')
        self.cfg = load_config(self.get_parameter('config_file').value or None)
        self.lock = threading.Lock()
        self.vstat = None
        self.cstate = None
        self.replies = []
        self.odom = None
        self.odom_t = 0.0
        self.ostat = None
        self.ostat_t = 0.0
        self.map_msg = None
        self.map_ver = 0
        self.map_t = 0.0
        self.scan = None
        self.scan_t = 0.0
        self.scan_times = []
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.pub_req = self.create_publisher(String, '/rc/command_request', 50)
        self.create_subscription(String, '/rc/vehicle_status', self._on_vstat, 10)
        self.create_subscription(String, '/rc/control_state', self._on_cstate, 10)
        self.create_subscription(String, '/rc/command_reply', self._on_reply, 50)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(String, '/rc/odom_status', self._on_ostat, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.serialize_cli = (self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
                              if SerializePoseGraph else None)

    def _on_vstat(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.vstat = d
            self.vstat_t = time.monotonic()

    def _on_cstate(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.cstate = d
            self.cstate_t = time.monotonic()

    def _on_reply(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.replies.append(d)

    def _on_odom(self, m):
        with self.lock:
            self.odom = m
            self.odom_t = time.monotonic()

    def _on_ostat(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.ostat = d
            self.ostat_t = time.monotonic()

    def _on_scan(self, m):
        t = time.monotonic()
        with self.lock:
            self.scan = m
            self.scan_t = t
            self.scan_times = [x for x in self.scan_times if t - x < 2.0] + [t]

    def _on_map(self, m):
        with self.lock:
            self.map_msg = m
            self.map_ver += 1
            self.map_t = time.monotonic()

    def request(self, d):
        self.pub_req.publish(String(data=dumps(d)))

    def lookup2d(self, target, source):
        """target←source 최신 2D 변환 (x, y, yaw, age_s) 또는 None."""
        try:
            tf = self.tf_buf.lookup_transform(target, source, Time())
        except Exception:
            return None
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        tr = tf.transform
        return tr.translation.x, tr.translation.y, _yaw(tr.rotation), age


# ===================================================================== WebSocket 최소 구현 (RFC 6455)
class WsClosed(Exception):
    pass


async def ws_read_message(reader, max_size=1 << 20):
    """완성된 (opcode, payload) 반환. 클라이언트 프레임은 반드시 마스킹되어 있어야 함."""
    parts, first_op = [], None
    while True:
        h = await reader.readexactly(2)
        fin, op = h[0] & 0x80, h[0] & 0x0F
        masked, ln = h[1] & 0x80, h[1] & 0x7F
        if ln == 126:
            ln = struct.unpack('!H', await reader.readexactly(2))[0]
        elif ln == 127:
            ln = struct.unpack('!Q', await reader.readexactly(8))[0]
        if ln > max_size or not masked:
            raise WsClosed('frame too large or unmasked')
        mask = await reader.readexactly(4)
        data = await reader.readexactly(ln)
        if ln:
            key = (mask * (ln // 4 + 1))[:ln]
            data = (int.from_bytes(data, 'big') ^ int.from_bytes(key, 'big')).to_bytes(ln, 'big')
        if op >= 0x8:                      # 제어 프레임
            return op, data
        if op != 0:
            first_op = op
        parts.append(data)
        if sum(len(p) for p in parts) > max_size:
            raise WsClosed('message too large')
        if fin:
            return first_op, b''.join(parts)


def ws_frame(payload, op=0x1):
    n = len(payload)
    if n < 126:
        hdr = struct.pack('!BB', 0x80 | op, n)
    elif n < 65536:
        hdr = struct.pack('!BBH', 0x80 | op, 126, n)
    else:
        hdr = struct.pack('!BBQ', 0x80 | op, 127, n)
    return hdr + payload


class Client:
    _ids = itertools.count(1)

    def __init__(self, writer, ip):
        self.id = f'c{next(self._ids)}'
        self.ip = ip
        self.writer = writer
        self.role = None
        self.name = ''
        self.queue = asyncio.Queue(maxsize=40)
        self.last_rx = time.monotonic()
        self.map_ver_sent = -1

    def send(self, obj_or_text, droppable=True):
        text = obj_or_text if isinstance(obj_or_text, str) else dumps(obj_or_text)
        try:
            self.queue.put_nowait(text)
        except asyncio.QueueFull:
            if not droppable:
                raise WsClosed('client too slow')


# ===================================================================== 서버
class WebServer:
    def __init__(self, node: BridgeNode):
        self.node = node
        self.cfg = node.cfg
        self.wc = self.cfg['web']
        self.nets = [ipaddress.ip_network(n) for n in self.wc['allowed_networks']]
        self.clients = {}
        self.map_cache = (None, None)      # (ver, json text)
        self.park_status = None            # (msg, 수신시각, autopark client id) — 노트북 자동주차 상태 중계
        self.park_paths = None             # 최근 경로 메시지(JSON 텍스트)
        from ament_index_python.packages import get_package_share_directory
        self.index_path = os.path.join(get_package_share_directory('rc_car'), 'web', 'index.html')

    def allowed(self, ip):
        try:
            a = ipaddress.ip_address(ip)
            if getattr(a, 'ipv4_mapped', None):
                a = a.ipv4_mapped
            return any(a in n for n in self.nets)
        except ValueError:
            return False

    async def handle(self, reader, writer):
        ip = (writer.get_extra_info('peername') or ('?',))[0]
        if not self.allowed(ip):
            self.node.get_logger().warn(f'허용되지 않은 IP 접속 거부: {ip}')
            writer.close()
            return
        try:
            req = await asyncio.wait_for(reader.readuntil(b'\r\n\r\n'), 10)
        except (asyncio.TimeoutError, asyncio.IncompleteReadError, asyncio.LimitOverrunError, OSError):
            writer.close()
            return
        lines = req.decode('latin-1').split('\r\n')
        try:
            method, path, _ = lines[0].split(' ', 2)
        except ValueError:
            writer.close()
            return
        hdr = {}
        for ln in lines[1:]:
            if ':' in ln:
                k, v = ln.split(':', 1)
                hdr[k.strip().lower()] = v.strip()
        path = path.split('?', 1)[0]
        if path == '/ws' and 'websocket' in hdr.get('upgrade', '').lower():
            await self.websocket(reader, writer, hdr, ip)
            return
        if method == 'GET' and path in ('/', '/index.html'):
            try:
                with open(self.index_path, 'rb') as f:
                    body = f.read()
                self._http(writer, '200 OK', 'text/html; charset=utf-8', body)
            except OSError:
                self._http(writer, '500 Internal Server Error', 'text/plain', b'index.html missing')
        else:
            self._http(writer, '404 Not Found', 'text/plain', b'not found')
        try:
            await writer.drain()
        except OSError:
            pass
        writer.close()

    @staticmethod
    def _http(writer, status, ctype, body):
        writer.write((f'HTTP/1.1 {status}\r\nContent-Type: {ctype}\r\nContent-Length: {len(body)}\r\n'
                      'Cache-Control: no-store\r\nConnection: close\r\n\r\n').encode() + body)

    async def websocket(self, reader, writer, hdr, ip):
        key = hdr.get('sec-websocket-key', '')
        accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        writer.write(('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                      f'Sec-WebSocket-Accept: {accept}\r\n\r\n').encode())
        await writer.drain()
        c = Client(writer, ip)
        self.clients[c.id] = c
        self.node.get_logger().info(f'WebSocket 연결 {c.id} ({ip})')
        sender = asyncio.ensure_future(self._sender(c))
        try:
            while True:
                op, data = await asyncio.wait_for(ws_read_message(reader), timeout=3.0)
                c.last_rx = time.monotonic()
                if op == 0x8:
                    break
                if op == 0x9:
                    writer.write(ws_frame(data, 0xA))
                    continue
                if op != 0x1:
                    continue
                try:
                    msg = json.loads(data.decode('utf-8'))
                except (ValueError, UnicodeDecodeError):
                    continue
                if isinstance(msg, dict):
                    await self.on_message(c, msg)
        except (asyncio.TimeoutError, asyncio.IncompleteReadError, WsClosed, OSError, ConnectionError) as e:
            self.node.get_logger().warn(f'{c.id} 연결 종료/타임아웃: {type(e).__name__}')
        finally:
            sender.cancel()
            self.clients.pop(c.id, None)
            if c.role:
                self.node.request({'client': c.id, 'role': c.role, 'type': 'bye'})
            try:
                writer.close()
            except Exception:
                pass
            self.node.get_logger().info(f'WebSocket 종료 {c.id}')

    async def _sender(self, c):
        try:
            while True:
                text = await c.queue.get()
                c.writer.write(ws_frame(text.encode('utf-8')))
                await asyncio.wait_for(c.writer.drain(), timeout=2.0)
        except (asyncio.CancelledError, asyncio.TimeoutError, OSError, ConnectionError):
            try:
                c.writer.close()
            except Exception:
                pass

    async def on_message(self, c, msg):
        typ = msg.get('type')
        if typ == 'hello':
            role = msg.get('role', 'ui')
            if role not in ('ui', 'autopark'):
                c.send({'type': 'reply', 'req': 'hello', 'ok': False, 'msg': 'role 은 ui 또는 autopark'})
                return
            c.role, c.name = role, str(msg.get('name', ''))[:40]
            self.node.request({'client': c.id, 'role': role, 'type': 'hello', 'name': c.name,
                               'ready': bool(msg.get('ready', False))})
            c.send({'type': 'welcome', 'id': c.id, 'role': role, 'config': self._ui_config()}, droppable=False)
            if role == 'ui' and self.park_paths:
                c.send(self.park_paths)
            return
        if c.role is None:
            c.send({'type': 'reply', 'req': typ, 'ok': False, 'msg': 'hello 먼저 전송'})
            return
        if typ == 'ping':
            c.send({'type': 'pong', 't': msg.get('t'), 'server_t': time.time()})
            self.node.request({'client': c.id, 'role': c.role, 'type': 'ping', 'ready': bool(msg.get('ready'))})
            return
        if typ == 'save_map':
            if c.role != 'ui':
                return
            asyncio.ensure_future(self._save_map(c, str(msg.get('name', ''))))
            return
        if typ in FORWARD:
            d = dict(msg)
            d.update({'client': c.id, 'role': c.role})
            self.node.request(d)
            return
        # ---- 자동주차 중계 (알고리즘은 노트북에서 실행, 브리지는 전달만) ----
        if typ == 'park_cmd' and c.role == 'ui':
            targets = [x for x in self.clients.values() if x.role == 'autopark']
            if not targets:
                c.send({'type': 'reply', 'req': 'park_cmd', 'ok': False,
                        'msg': '노트북 자동주차 프로그램(python -m autopark run) 미연결'})
                return
            d = dict(msg)
            d['from'] = c.id
            for x in targets:
                x.send(d, droppable=False)
            return
        if typ in ('park_status', 'park_paths') and c.role == 'autopark':
            text = dumps(msg)
            if typ == 'park_status':
                self.park_status = (msg, time.monotonic(), c.id)
            else:
                self.park_paths = text
                for x in self.clients.values():
                    if x.role == 'ui':
                        x.send(text)
            return
        c.send({'type': 'reply', 'req': typ, 'ok': False, 'msg': f'알 수 없는 type: {typ}'})

    def _ui_config(self):
        cf = self.cfg
        return {
            'wheel_diameter_m': cf['vehicle']['wheel_diameter_m'],
            'wheelbase_m': cf['vehicle']['wheelbase_m'],
            'track_width_m': cf['vehicle']['track_width_m'],
            'rated_rpm': cf['motors']['rated_rpm'],
            'lidar': {k: cf['lidar'][k] for k in ('x', 'y', 'yaw')},
            'ultrasonic': [{'name': s['name'], 'label': s.get('label', s['name'])}
                           for s in cf['ultrasonic']['sensors']],
            'ultrasonic_max_m': cf['ultrasonic']['max_range_m'],
            'command_timeout_s': cf['safety']['command_timeout_s'],
            'autopark_enabled': bool(cf['autopark'].get('enabled')),
        }

    # ------------------------------------------------------------- 지도 저장
    async def _save_map(self, c, name):
        name = ''.join(ch for ch in name if ch.isalnum() or ch in '-_') or time.strftime('map_%Y%m%d_%H%M%S')
        mdir = self.cfg['slam']['maps_dir']
        base = os.path.join(mdir, name)
        result = {'type': 'reply', 'req': 'save_map', 'ok': False, 'msg': ''}
        with self.node.lock:
            m = self.node.map_msg
        if m is None:
            result['msg'] = '/map 미수신 — 저장할 지도가 없음'
            c.send(result, droppable=False)
            return
        try:
            os.makedirs(mdir, exist_ok=True)
            await asyncio.get_event_loop().run_in_executor(None, save_pgm_yaml, m, base)
            notes = [f'{name}.pgm/.yaml 저장']
        except OSError as e:
            result['msg'] = f'PGM 저장 실패: {e}'
            c.send(result, droppable=False)
            return
        cli = self.node.serialize_cli
        if cli is None:
            notes.append('slam_toolbox.srv 없음 → posegraph 미저장')
        elif not cli.service_is_ready():
            notes.append('/slam_toolbox/serialize_map 서비스 없음 → posegraph 미저장')
        else:
            req = SerializePoseGraph.Request()
            req.filename = base
            fut = cli.call_async(req)
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 15:
                await asyncio.sleep(0.1)
            if fut.done() and fut.result() is not None and fut.result().result == 0:
                notes.append(f'{name}.posegraph/.data 저장 (재사용 가능)')
            else:
                notes.append('posegraph 저장 실패/시간초과')
        result.update({'ok': True, 'msg': ', '.join(notes), 'path': base})
        c.send(result, droppable=False)

    # ------------------------------------------------------------- 주기 송신
    def _scan_points(self, scan):
        frame = scan.header.frame_id or 'laser'
        for target in ('map', 'odom'):
            tf = self.node.lookup2d(target, frame)
            if tf:
                break
        else:
            return None
        x0, y0, yaw, _age = tf
        c, s = math.cos(yaw), math.sin(yaw)
        n = len(scan.ranges)
        step = max(1, n // 360)
        pts = []
        for i in range(0, n, step):
            r = scan.ranges[i]
            if not (scan.range_min <= r <= scan.range_max):
                continue
            a = scan.angle_min + i * scan.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            pts.append(round(x0 + c * lx - s * ly, 3))
            pts.append(round(y0 + s * lx + c * ly, 3))
        return {'type': 'scan', 'frame': target, 'pts': pts}

    def _state(self):
        n = self.node
        now = time.monotonic()
        with n.lock:
            vstat, cstate = n.vstat, n.cstate
            vt, ct = getattr(n, 'vstat_t', 0.0), getattr(n, 'cstate_t', 0.0)
            odom, odom_t = n.odom, n.odom_t
            ostat = n.ostat if now - n.ostat_t < 1.0 else None
            scan_t, scan_hz = n.scan_t, len(n.scan_times) / 2.0
            map_msg, map_t = n.map_msg, n.map_t
            replies, n.replies = n.replies, []
        pose = None
        tf_map = n.lookup2d('map', 'base_link')
        tf_odom = n.lookup2d('odom', 'base_link')
        if tf_map and tf_map[3] < 1.0:
            pose = {'frame': 'map', 'x': tf_map[0], 'y': tf_map[1], 'yaw': tf_map[2], 'age_s': max(0.0, tf_map[3])}
        elif tf_odom and tf_odom[3] < 1.0:
            pose = {'frame': 'odom', 'x': tf_odom[0], 'y': tf_odom[1], 'yaw': tf_odom[2], 'age_s': max(0.0, tf_odom[3])}
        if pose:
            pose = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in pose.items()}
        od = None
        if odom is not None and now - odom_t < 1.0:
            od = {'x': round(odom.pose.pose.position.x, 4), 'y': round(odom.pose.pose.position.y, 4),
                  'yaw': round(_yaw(odom.pose.pose.orientation), 4),
                  'v': round(odom.twist.twist.linear.x, 3), 'w': round(odom.twist.twist.angular.z, 3)}
        lidar_ok = now - scan_t < 1.0
        odom_ok = od is not None and bool(ostat and ostat.get('ok'))
        map_tf_ok = bool(tf_map and tf_map[3] < 1.0)
        map_ok = map_msg is not None
        slam = {'lidar_ok': lidar_ok, 'scan_hz': scan_hz, 'odom_ok': odom_ok, 'map_tf_ok': map_tf_ok,
                'map_received': map_ok, 'map_age_s': round(now - map_t, 1) if map_ok else None,
                'odom_status': ostat,
                # 라이다·라이다 odom·TF·지도가 모두 정상일 때만 SLAM 동작 중으로 표시
                'ready': lidar_ok and odom_ok and map_tf_ok and map_ok}
        return {
            'type': 'state', 't': time.time(),
            'vehicle': vstat if vstat and now - vt < 1.0 else None,
            'control': cstate if cstate and now - ct < 1.0 else None,
            'pose': pose, 'odom': od, 'slam': slam,
            'clients': len(self.clients),
            'park': self._park_state(now),
        }, replies

    def _park_state(self, now):
        ps = self.park_status
        connected = any(x.role == 'autopark' for x in self.clients.values())
        if ps is None or ps[2] not in self.clients:
            return {'connected': connected, 'status': None}
        return {'connected': connected, 'status': ps[0], 'age_s': round(now - ps[1], 2)}

    async def broadcaster(self):
        period = 1.0 / self.wc['state_rate_hz']
        scan_period = 1.0 / self.wc['scan_rate_hz']
        last_scan = 0.0
        last_map = 0.0
        loop = asyncio.get_event_loop()
        while True:
            await asyncio.sleep(period)
            now = time.monotonic()
            try:
                state, replies = self._state()
            except Exception as e:  # 상태 조립 실패가 서버를 죽이지 않도록
                self.node.get_logger().error(f'state 생성 오류: {e}')
                continue
            for r in replies:
                c = self.clients.get(r.get('client'))
                if c:
                    c.send({'type': 'reply', 'req': r.get('type'), 'ok': r.get('ok'), 'msg': r.get('msg')})
            ctrl = state.get('control') or {}
            for c in list(self.clients.values()):
                if c.role is None:
                    continue
                state['you'] = {'id': c.id, 'role': c.role, 'owner': ctrl.get('owner') == c.id}
                c.send(dumps(state))
            if now - last_scan >= scan_period:
                last_scan = now
                with self.node.lock:
                    scan = self.node.scan if now - self.node.scan_t < 1.0 else None
                if scan is not None:
                    sp = self._scan_points(scan)
                    if sp:
                        text = dumps(sp)
                        for c in self.clients.values():
                            if c.role:
                                c.send(text)
            with self.node.lock:
                ver, m = self.node.map_ver, self.node.map_msg
            need = [c for c in self.clients.values() if c.role and c.map_ver_sent != ver]
            if m is not None and need and (now - last_map >= self.wc['map_min_interval_s']
                                           or any(c.map_ver_sent < 0 for c in need)):
                last_map = now
                if self.map_cache[0] != ver:
                    text = await loop.run_in_executor(None, encode_map, m)
                    self.map_cache = (ver, text)
                for c in need:
                    c.send(self.map_cache[1])
                    c.map_ver_sent = ver


def encode_map(m):
    data = m.data.tobytes() if hasattr(m.data, 'tobytes') else bytes((v & 0xFF) for v in m.data)
    info = m.info
    o = info.origin
    return dumps({'type': 'map', 'frame': m.header.frame_id or 'map',
                  'width': info.width, 'height': info.height, 'resolution': info.resolution,
                  'origin': {'x': o.position.x, 'y': o.position.y, 'yaw': _yaw(o.orientation)},
                  'encoding': 'int8-zlib-base64',
                  'data': base64.b64encode(zlib.compress(data, 6)).decode('ascii')})


_PGM_LUT = bytes((205 if v < 0 else 0 if v > 65 else 254 if v < 20 else 205)
                 for v in (u if u < 128 else u - 256 for u in range(256)))


def save_pgm_yaml(m, base):
    """map_server 호환 PGM + YAML (trinary)."""
    w, h = m.info.width, m.info.height
    raw = m.data.tobytes() if hasattr(m.data, 'tobytes') else bytes((v & 0xFF) for v in m.data)
    img = raw.translate(_PGM_LUT)
    rows = [img[r * w:(r + 1) * w] for r in range(h - 1, -1, -1)]   # PGM 은 위쪽이 +y
    with open(base + '.pgm', 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(b''.join(rows))
    o = m.info.origin
    with open(base + '.yaml', 'w', encoding='utf-8') as f:
        f.write(f'image: {os.path.basename(base)}.pgm\nmode: trinary\nresolution: {m.info.resolution}\n'
                f'origin: [{o.position.x}, {o.position.y}, {_yaw(o.orientation)}]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


def main():
    rclpy.init()
    node = BridgeNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    server = WebServer(node)
    node.request({'type': 'reset_clients'})

    async def run():
        srv = await asyncio.start_server(server.handle, '0.0.0.0', server.wc['port'])
        node.get_logger().info(f'웹 UI: http://<라즈베리파이IP>:{server.wc["port"]}  (WebSocket /ws)')
        asyncio.ensure_future(server.broadcaster())
        async with srv:
            await srv.serve_forever()

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
