"""Command Manager — 제어권/모드/안전 관리. 수동 조종(웹 UI)과 자동주차(노트북)는 같은 요청 규격을 쓰고,
여기서 하나의 최종 명령 /rc/drive_cmd 로 합쳐져 공통 차량 제어부(vehicle_node)로 간다.

  입력  /rc/command_request (String JSON: {client, role, type, ...}) ← web_bridge
        /rc/vehicle_status, /odom, TF map→base_link
  출력  /rc/drive_cmd (20 Hz), /rc/control_state (10 Hz), /rc/command_reply, /rc/settings, /rc/hw_request

규칙
  * E-STOP: 모든 클라이언트가 걸 수 있음(래치). 해제는 UI 클라이언트만.
  * MANUAL: 제어권(owner)을 가진 UI 한 곳만 주행 명령 가능 → 동시 제어 충돌 방지.
  * AUTO_PARK: config autopark.enabled=true + 자동주차 클라이언트 ready + 피드백(엔코더/odom/SLAM 위치) 유효
               일 때만 진입. 진입 후에는 autopark 클라이언트의 속도(m/s)/조향(rad) 명령만 수용 (Open-loop 불가).
               피드백 손실·클라이언트 끊김 → 즉시 정지 후 MANUAL 로 복귀 (Open-loop 로 자동 전환하지 않음).
  * 모드 전환: 먼저 정지 명령 → 실제 정지 확인 → 전환. 제한시간 내 정지 못하면 전환 취소.
  * 명령 타임아웃: 제어 주체의 마지막 명령이 safety.command_timeout_s 보다 오래되면 정지.
"""
import collections
import json
import math
import time

import rclpy
import rclpy.executors
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .common import dumps, load_config

MANUAL, AUTO = 'MANUAL', 'AUTO_PARK'


def _finite(x):
    return isinstance(x, (int, float)) and math.isfinite(x)


class CommandManager(Node):
    def __init__(self):
        super().__init__('rc_command_manager')
        self.declare_parameter('config_file', '')
        self.cfg = load_config(self.get_parameter('config_file').value or None)
        self.safety = self.cfg['safety']
        self.ap = self.cfg['autopark']
        self.mode = MANUAL
        self.pending = None            # (target_mode, since, reason)
        self.stopped_since = None
        self.estop = False
        self.control_mode = self.cfg['speed_control'].get('default_mode', 'PI')
        self.owner = None
        self.clients = {}              # id → {role, name, seen, ready}
        self.last_drive = None         # (client, t, payload)
        self.last_nonstop_t = 0.0
        self.vstat = {}
        self.vstat_t = 0.0
        self.odom_t = 0.0
        self.events = collections.deque(maxlen=12)
        self.seq = 0
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.pub_cmd = self.create_publisher(String, '/rc/drive_cmd', 10)
        self.pub_state = self.create_publisher(String, '/rc/control_state', 10)
        self.pub_reply = self.create_publisher(String, '/rc/command_reply', 50)
        self.pub_settings = self.create_publisher(String, '/rc/settings', 10)
        self.pub_hw = self.create_publisher(String, '/rc/hw_request', 10)
        self.create_subscription(String, '/rc/command_request', self._on_req, 50)
        self.create_subscription(String, '/rc/vehicle_status', self._on_vstat, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_timer(0.05, self._tick)
        self.create_timer(0.1, self._publish_state)
        self._event('Command Manager 시작 — MANUAL')

    # ------------------------------------------------------------------ helpers
    def _event(self, text):
        self.events.append({'t': round(time.time(), 2), 'msg': text})
        self.get_logger().info(text)

    def _reply(self, client, typ, ok, msg=''):
        self.pub_reply.publish(String(data=dumps({'client': client, 'type': typ, 'ok': ok, 'msg': msg})))

    def _speed(self):
        if time.monotonic() - self.vstat_t > 1.0:
            return None
        return self.vstat.get('speed_m_s')

    def _map_pose_ok(self):
        try:
            tf = self.tf_buf.lookup_transform('map', 'base_link', Time())
        except Exception:
            return False
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        return age < 1.0

    def _feedback(self):
        now = time.monotonic()
        fb = {
            'vehicle_status': now - self.vstat_t < self.ap['feedback_timeout_s'] * 2,
            'encoders_ok': bool(self.vstat.get('encoders_ok')) and now - self.vstat_t < 1.0,
            'odom': now - self.odom_t < self.ap['feedback_timeout_s'],
            'map_pose': self._map_pose_ok(),
            # 트레일러 후진 주차는 MT6701 연결각 실측이 필수
            'trailer_angle': now - self.vstat_t < 1.0 and
            (self.vstat.get('trailer') or {}).get('angle_rad') is not None,
        }
        return fb

    def _autopark_client(self):
        now = time.monotonic()
        for cid, c in self.clients.items():
            if c['role'] == 'autopark' and now - c['seen'] < self.ap['feedback_timeout_s'] * 4:
                return cid, c
        return None, None

    def _autopark_block_reasons(self):
        r = []
        if not self.ap.get('enabled'):
            r.append('config autopark.enabled=false (자동주차 알고리즘 미준비)')
        cid, c = self._autopark_client()
        if cid is None:
            r.append('자동주차 클라이언트 미연결')
        elif not c.get('ready'):
            r.append('자동주차 클라이언트 ready=false')
        if self.estop:
            r.append('E-STOP 활성')
        if self.control_mode != 'PI':
            r.append('속도제어 모드가 PI 가 아님')
        fb = self._feedback()
        if not fb['encoders_ok']:
            r.append('엔코더 피드백 없음/고장')
        if not fb['odom']:
            r.append('/odom 없음')
        if self.ap.get('require_map_pose', True) and not fb['map_pose']:
            r.append('SLAM 위치(map→base_link) 없음')
        if not fb['trailer_angle']:
            r.append('MT6701 트레일러 연결각 없음')
        if not self.vstat.get('motors_enabled'):
            r.append('모터 비활성 상태')
        return r

    def _is_ui(self, cid):
        return self.clients.get(cid, {}).get('role') == 'ui'

    def _can_configure(self, cid):
        return self._is_ui(cid) and (self.owner is None or self.owner == cid)

    def _request_mode(self, target, reason):
        if target == self.mode and self.pending is None:
            return
        self.pending = (target, time.monotonic(), reason)
        self.stopped_since = None
        self.last_drive = None
        self._event(f'모드 전환 요청 → {target} ({reason}) : 정지 중')

    # ------------------------------------------------------------------ requests
    def _on_vstat(self, m):
        try:
            self.vstat = json.loads(m.data)
            self.vstat_t = time.monotonic()
        except ValueError:
            pass

    def _on_odom(self, _m):
        self.odom_t = time.monotonic()

    def _on_req(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        cid, typ = d.get('client'), d.get('type')
        now = time.monotonic()
        if typ == 'reset_clients':                       # web_bridge 재시작
            self.clients.clear()
            self.owner = None
            self.last_drive = None
            return
        if typ == 'hello':
            self.clients[cid] = {'role': d.get('role', 'ui'), 'name': d.get('name', ''), 'seen': now,
                                 'ready': bool(d.get('ready', False))}
            return
        if cid not in self.clients:
            self._reply(cid, typ, False, 'hello 먼저 필요')
            return
        cl = self.clients[cid]
        cl['seen'] = now
        role = cl['role']

        if typ == 'bye':
            self.clients.pop(cid, None)
            if self.owner == cid:
                self.owner = None
                self._event(f'제어권 해제 (연결 종료: {cid})')
            if self.last_drive and self.last_drive[0] == cid:
                self.last_drive = None
            return
        if typ == 'ping':
            if role == 'autopark':
                cl['ready'] = bool(d.get('ready', False))
            return
        if typ == 'estop':
            if not self.estop:
                self._event(f'E-STOP ({cid})')
            self.estop = True
            self.last_drive = None
            self._reply(cid, typ, True)
            return
        if typ == 'stop':
            self.last_drive = None
            self._reply(cid, typ, True)
            return
        if typ == 'estop_reset':
            if role != 'ui':
                return self._reply(cid, typ, False, 'E-STOP 해제는 웹 UI 에서만 가능')
            self.estop = False
            self.last_drive = None
            self.pub_hw.publish(String(data=dumps({'type': 'fault_reset'})))
            self._event(f'E-STOP 해제 / 고장 리셋 ({cid})')
            return self._reply(cid, typ, True)
        if typ == 'claim':
            if role != 'ui':
                return self._reply(cid, typ, False, 'UI 만 수동 제어권을 가질 수 있음')
            idle = self.last_drive is None or self.last_drive[0] != self.owner or \
                now - self.last_drive[1] > 2.0
            if self.owner in (None, cid) or self.owner not in self.clients or (d.get('force') and idle):
                if self.owner != cid:
                    self._event(f'수동 제어권 → {cid}')
                self.owner = cid
                return self._reply(cid, typ, True)
            return self._reply(cid, typ, False, f'다른 UI({self.owner})가 제어 중')
        if typ == 'release':
            if self.owner == cid:
                self.owner = None
                self.last_drive = None
                self._event(f'수동 제어권 해제 ({cid})')
            return self._reply(cid, typ, True)
        if typ == 'mode':
            target = d.get('mode')
            if target == AUTO:
                reasons = self._autopark_block_reasons()
                if self.mode == AUTO:
                    return self._reply(cid, typ, True)
                if reasons:
                    return self._reply(cid, typ, False, 'AUTO_PARK 불가: ' + '; '.join(reasons))
                self._request_mode(AUTO, f'요청: {cid}')
                return self._reply(cid, typ, True, '정지 확인 후 전환')
            if target == MANUAL:
                self._request_mode(MANUAL, f'요청: {cid}')
                return self._reply(cid, typ, True)
            return self._reply(cid, typ, False, '알 수 없는 모드')
        if typ == 'drive':
            return self._on_drive(cid, role, d, now)
        if typ == 'set_control_mode':
            cm = d.get('control_mode')
            if cm not in ('PI', 'OPEN_LOOP'):
                return self._reply(cid, typ, False, 'PI 또는 OPEN_LOOP')
            if self.mode != MANUAL or not self._can_configure(cid):
                return self._reply(cid, typ, False, 'MANUAL 모드의 제어권 UI 만 가능')
            sp = self._speed()
            idle = now - self.last_nonstop_t > 1.0
            if (sp is not None and abs(sp) > self.safety['stop_speed_m_s']) or not idle:
                return self._reply(cid, typ, False, '정지 상태에서만 전환 가능')
            if cm != self.control_mode:
                self._event(f'속도제어 모드 {self.control_mode} → {cm}')
            self.control_mode = cm
            self.last_drive = None
            return self._reply(cid, typ, True)
        if typ == 'set_limits':
            if not self._can_configure(cid):
                return self._reply(cid, typ, False, '제어권 UI 만 가능')
            lim = {k: float(d[k]) for k in ('max_rpm', 'max_pwm', 'max_steer_rad') if _finite(d.get(k))}
            self.pub_settings.publish(String(data=dumps(lim)))
            return self._reply(cid, typ, True)
        if typ == 'trailer_zero':
            if not self._can_configure(cid):
                return self._reply(cid, typ, False, '제어권 UI 만 가능')
            self.pub_hw.publish(String(data=dumps({'type': 'trailer_zero'})))
            self._event('트레일러 영점 설정')
            return self._reply(cid, typ, True)
        self._reply(cid, typ, False, f'알 수 없는 type: {typ}')

    def _on_drive(self, cid, role, d, now):
        steer = d.get('steer', 0.0)
        if not _finite(steer):
            return self._reply(cid, 'drive', False, 'steer 는 유한한 실수(rad)')
        if self.estop:
            return  # E-STOP 중 주행 명령 무시 (20 Hz 반복 전송이므로 응답 생략)
        if self.pending is not None:
            return
        if self.mode == MANUAL:
            if role != 'ui':
                return self._reply(cid, 'drive', False, 'MANUAL 모드: 자동주차 클라이언트 명령 거부')
            if self.owner is None:
                self.owner = cid
                self._event(f'수동 제어권 → {cid}')
            if self.owner != cid:
                return
            if self.control_mode == 'PI':
                if not _finite(d.get('v')):
                    return self._reply(cid, 'drive', False, 'PI 모드: v(m/s) 필요')
                payload = {'mode': 'velocity', 'v': float(d['v']), 'steer': float(steer)}
            else:
                thr = d.get('throttle')
                if not _finite(thr) or abs(thr) > 1.0:
                    return self._reply(cid, 'drive', False, 'OPEN_LOOP 모드: throttle(-1~1) 필요')
                payload = {'mode': 'throttle', 'throttle': float(thr), 'steer': float(steer)}
        else:
            ap_id, _ = self._autopark_client()
            if role != 'autopark' or cid != ap_id:
                return
            if not _finite(d.get('v')):
                return self._reply(cid, 'drive', False, 'AUTO_PARK: v(m/s) 필요')
            payload = {'mode': 'velocity', 'v': float(d['v']), 'steer': float(steer)}
        self.last_drive = (cid, now, payload)

    # ------------------------------------------------------------------ 20 Hz
    def _tick(self):
        now = time.monotonic()
        # 응답 없는 autopark 클라이언트 제거 (UI 는 bye 로 제거)
        for cid in [k for k, c in self.clients.items()
                    if c['role'] == 'autopark' and now - c['seen'] > 5.0]:
            self.clients.pop(cid)
            self._event(f'자동주차 클라이언트 타임아웃: {cid}')

        out = {'mode': 'stop', 'v': 0.0, 'throttle': 0.0, 'steer': None, 'source': None}
        if self.mode == AUTO and self.pending is None:
            reasons = []
            fb = self._feedback()
            ap_id, ap = self._autopark_client()
            if ap_id is None:
                reasons.append('자동주차 클라이언트 끊김')
            elif not ap.get('ready'):
                reasons.append('자동주차 클라이언트 ready 해제')
            if not fb['encoders_ok'] or not fb['odom']:
                reasons.append('엔코더/odom 피드백 손실')
            if self.ap.get('require_map_pose', True) and not fb['map_pose']:
                reasons.append('SLAM 위치 손실')
            if not fb['trailer_angle']:
                reasons.append('트레일러 연결각 손실')
            if not self.ap.get('enabled'):
                reasons.append('autopark 비활성')
            if reasons:
                self._request_mode(MANUAL, '자동 정지: ' + ', '.join(reasons))

        if self.pending is not None:
            target, since, _r = self.pending
            sp = self._speed()
            if sp is not None and abs(sp) <= self.safety['stop_speed_m_s']:
                self.stopped_since = self.stopped_since or now
            else:
                self.stopped_since = None
            # 엔코더가 없으면(sp None) 속도 확인 불가 → 정지 출력 유지 후 제한시간 경과 시 MANUAL 로만 전환
            if self.stopped_since and now - self.stopped_since > 0.3:
                if target == AUTO and self._autopark_block_reasons():
                    self._event('AUTO_PARK 전환 취소: 조건 불충족')
                else:
                    self.mode = target
                    self._event(f'모드 = {target}')
                self.pending = None
            elif now - since > self.safety['mode_switch_timeout_s']:
                if target == MANUAL:
                    self.mode = MANUAL
                    self._event('정지 확인 불가 — 정지 명령 상태로 MANUAL 전환')
                else:
                    self._event('정지 확인 실패 — AUTO_PARK 전환 취소')
                self.pending = None
        elif not self.estop and self.last_drive is not None:
            cid, t, payload = self.last_drive
            if now - t <= self.safety['command_timeout_s']:
                out.update(payload)
                out['source'] = cid
            else:
                self.last_drive = None
                self._event(f'명령 타임아웃 → 정지 ({cid})')

        if out['mode'] != 'stop':
            self.last_nonstop_t = now
        if out['steer'] is None:   # 정지 시 조향은 현재 명령 유지 (불필요한 서보 움직임 방지)
            out.pop('steer')
            prev = getattr(self, '_last_steer', 0.0)
            out['steer'] = prev
        self._last_steer = out['steer']
        self.seq += 1
        out.update({'seq': self.seq, 'estop': self.estop, 'control_mode': self.control_mode,
                    'vehicle_mode': self.mode})
        self.pub_cmd.publish(String(data=dumps(out)))
        self._out = out

    def _publish_state(self):
        ap_id, ap = self._autopark_client()
        reasons = self._autopark_block_reasons() if self.mode == MANUAL else []
        st = {
            'mode': self.mode,
            'pending_mode': self.pending[0] if self.pending else None,
            'estop': self.estop,
            'control_mode': self.control_mode,
            'owner': self.owner,
            'clients': [{'id': k, 'role': c['role'], 'name': c['name']} for k, c in self.clients.items()],
            'autopark': {'enabled': bool(self.ap.get('enabled')), 'client': ap_id,
                         'ready': bool(ap and ap.get('ready')), 'allowed': not reasons,
                         'block_reasons': reasons},
            'feedback': self._feedback(),
            'output': getattr(self, '_out', None),
            'events': list(self.events),
        }
        self.pub_state.publish(String(data=dumps(st)))


def main():
    rclpy.init()
    node = CommandManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
