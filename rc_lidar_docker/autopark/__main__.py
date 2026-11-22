"""노트북 실행 진입점.

  python -m autopark run <Pi-IP> [--port 8080] [--config autopark/config.yaml]
  python -m autopark plan (--map maps/room1.yaml | --empty 6x4) --start X Y YAW BETA --goal X Y YAW [--svg out.svg]

run: 기존 Pi 웹 브리지(ws://Pi:8080/ws)에 role=autopark 로 접속. 웹 UI 에서 목표를 찍고 '경로 계획'/'자동주차 시작'을
     누르면 이 프로그램이 계획·추종하고, 결과(경로·상태)를 같은 WebSocket 으로 UI 에 보낸다.
     차량 구동은 기존 drive{v, steer} → Command Manager → 공통 차량 제어부 경로만 사용 (PWM 직접 제어 없음).
plan: 저장된 지도(.yaml/.pgm) 또는 빈 공간에서 경로 계획만 수행 (차량 구동 없음, 알고리즘 검증용).
"""
import argparse
import base64
import json
import math
import os
import socket
import struct
import sys
import threading
import time

from . import GridMap, TrackingMPC, geometry_from_config, load_config, plan_parking
from .model import Footprint, trailer_pose, wrap


# ====================================================================== WebSocket 클라이언트 (표준 라이브러리)
class WS:
    def __init__(self, host, port, path='/ws'):
        self.s = socket.create_connection((host, port), timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        self.s.sendall((f'GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n'
                        f'Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n')
                       .encode())
        resp = b''
        while b'\r\n\r\n' not in resp:
            chunk = self.s.recv(1024)
            if not chunk:
                raise ConnectionError('핸드셰이크 실패')
            resp += chunk
        if b' 101 ' not in resp.split(b'\r\n')[0]:
            raise ConnectionError(resp.split(b'\r\n')[0].decode(errors='replace'))
        self.buf = resp.split(b'\r\n\r\n', 1)[1]
        self.s.settimeout(None)
        self.lock = threading.Lock()

    def send(self, obj):
        data = json.dumps(obj, ensure_ascii=False).encode()
        n = len(data)
        if n < 126:
            hdr = bytes([0x81, 0x80 | n])
        elif n < 65536:
            hdr = bytes([0x81, 0x80 | 126]) + struct.pack('!H', n)
        else:
            hdr = bytes([0x81, 0x80 | 127]) + struct.pack('!Q', n)
        mask = os.urandom(4)
        key = (mask * (n // 4 + 1))[:n]
        body = (int.from_bytes(data, 'big') ^ int.from_bytes(key, 'big')).to_bytes(n, 'big') if n else b''
        with self.lock:
            self.s.sendall(hdr + mask + body)

    def _read(self, n):
        while len(self.buf) < n:
            chunk = self.s.recv(65536)
            if not chunk:
                raise ConnectionError('연결 끊김')
            self.buf += chunk
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def recv(self):
        parts = []
        while True:
            b0, b1 = self._read(2)
            op, n = b0 & 0x0F, b1 & 0x7F
            if n == 126:
                n = struct.unpack('!H', self._read(2))[0]
            elif n == 127:
                n = struct.unpack('!Q', self._read(8))[0]
            data = self._read(n)
            if op == 0x8:
                raise ConnectionError('서버가 연결 종료')
            if op in (0x9, 0xA):
                continue
            parts.append(data)
            if b0 & 0x80:
                return json.loads(b''.join(parts).decode())

    def close(self):
        try:
            self.s.close()
        except OSError:
            pass


# ====================================================================== 피드백 유효성 (가상값 사용 금지)
class FeedbackMonitor:
    """Pi state 메시지에서 실제 측정값과 그 유효성/나이를 판정. 측정 안 된 값은 None 으로 둔다."""

    def __init__(self, cfg):
        self.c = cfg['feedback']

    def evaluate(self, st, recv_t, now):
        c = self.c
        rep = {'checks': {}, 'values': {}, 'motion_ok': False, 'planning_start_ok': False, 'reasons': []}
        chk = rep['checks']
        if st is None:
            rep['reasons'].append('Pi 상태 미수신')
            return rep
        link_age = now - recv_t
        chk['link'] = {'ok': link_age <= c['max_state_age_s'], 'age_s': round(link_age, 3)}
        pose = st.get('pose')
        if pose and pose.get('frame') == 'map' and pose.get('age_s') is not None:
            age = pose['age_s'] + link_age
            chk['pose'] = {'ok': age <= c['max_pose_age_s'], 'age_s': round(age, 3), 'source': 'SLAM map→base_link'}
        elif pose and pose.get('frame') == 'odom':
            chk['pose'] = {'ok': False, 'source': 'odom 만 있음', 'reason': 'SLAM 위치(map 프레임) 없음'}
        else:
            chk['pose'] = {'ok': False, 'reason': '위치 없음 (SLAM/odom TF 미수신 — 엔코더 odom 이 없으면 SLAM 도 위치를 못 냄)'}
        if pose and chk['pose'].get('ok'):
            rep['values'].update({'x': pose['x'], 'y': pose['y'], 'th': pose['yaw']})
        veh = st.get('vehicle')
        vt_ok = False
        if veh and veh.get('stamp') and st.get('t'):
            vage = st['t'] - veh['stamp'] + link_age
            vt_ok = vage <= c['max_status_age_s']
            chk['vehicle_status'] = {'ok': vt_ok, 'age_s': round(vage, 3)}
        else:
            chk['vehicle_status'] = {'ok': False, 'reason': '차량 상태 없음'}
        tr = (veh or {}).get('trailer') or {}
        if vt_ok and tr.get('enabled') and tr.get('angle_rad') is not None:
            chk['trailer'] = {'ok': True, 'zero_source': tr.get('zero_source')}
            rep['values']['beta'] = tr['angle_rad']
        else:
            chk['trailer'] = {'ok': False, 'reason': 'MT6701 각도 없음 (trailer.ppr 미설정 또는 상태 지연)'}
        if vt_ok and veh.get('encoders_ok') and veh.get('speed_m_s') is not None:
            chk['speed'] = {'ok': True, 'source': 'DC 모터 엔코더'}
            rep['values']['v'] = veh['speed_m_s']
        else:
            chk['speed'] = {'ok': not c['require_encoder_speed'],
                            'reason': '엔코더 속도 피드백 없음/고장 (대체값 사용 안 함)'}
        st_cal = bool(((veh or {}).get('steering') or {}).get('calibrated'))
        chk['steering'] = {'ok': st_cal or not c['require_steering_calibrated'],
                           'reason': '' if st_cal else '조향 미보정 (vehicle.yaml steering.calibrated)'}
        ctrl = st.get('control') or {}
        ap = ctrl.get('autopark') or {}
        chk['pi_autopark_enabled'] = {'ok': bool(ap.get('enabled')),
                                      'reason': '' if ap.get('enabled') else 'Pi config autopark.enabled=false (하드웨어 검증 전)'}
        chk['estop'] = {'ok': not ctrl.get('estop', False)}
        us = []
        for u in (veh or {}).get('ultrasonic', []):
            if u.get('status') == 'OK' and u.get('range_m') is not None and (u.get('age_s') or 9) < 0.5:
                us.append(u['range_m'])
        rep['values']['us_min'] = min(us) if us else None
        for k, v in chk.items():
            if not v.get('ok'):
                rep['reasons'].append(f'{k}: {v.get("reason", "무효")}')
        rep['motion_ok'] = all(v.get('ok') for v in chk.values())
        rep['planning_start_ok'] = chk['pose']['ok'] and chk['trailer']['ok']
        return rep


# ====================================================================== 자동주차 상태기계
class AutoParkController:
    """IDLE → PLANNING → PLANNED → WAIT_AUTO → TRACKING (↔ REPLANNING) → PARKED | STOPPED | PLAN_FAILED"""

    def __init__(self, cfg, link):
        self.cfg = cfg
        self.link = link
        self.mon = FeedbackMonitor(cfg)
        self.lock = threading.RLock()
        self.geom, self.missing = geometry_from_config(cfg)
        self.phase = 'CONFIG_INCOMPLETE' if self.geom is None else 'IDLE'
        self.message = ('설정 누락: ' + ', '.join(self.missing)) if self.geom is None else '대기'
        self.state, self.state_t = None, 0.0
        self.grid, self.map_id = None, None
        self.goal = None
        self.plan = None            # dict: astar, traj, goal_state, start, validation_only, grid, id
        self.plan_counter = 0
        self.tracker = TrackingMPC(self.geom, cfg) if self.geom else None
        self.track_out = None
        self.replans = 0
        self.wait_t = 0.0
        self.last_reply = {}
        self.planning = False
        self.feedback = self.mon.evaluate(None, 0, 0)
        self.notes = []

    # ---------------- 입력
    def on_state(self, st):
        with self.lock:
            self.state, self.state_t = st, time.monotonic()

    def on_map(self, m):
        def build():
            try:
                g = GridMap.from_ws_map(m, self.cfg['map'])
                with self.lock:
                    self.grid = g
                    self.map_id = (m['width'], m['height'], time.monotonic())
            except Exception as e:  # 지도 파싱 실패는 계획만 막음
                self._set('IDLE' if self.phase == 'IDLE' else self.phase, f'지도 변환 실패: {e}')
        threading.Thread(target=build, daemon=True).start()

    def on_reply(self, m):
        self.last_reply[m.get('req')] = m
        if m.get('req') == 'mode' and not m.get('ok') and self.phase == 'WAIT_AUTO':
            self.stop(f'AUTO_PARK 전환 거부: {m.get("msg")}')

    def on_park_cmd(self, m):
        a = m.get('action')
        if a == 'plan':
            self.request_plan(m.get('goal'), m.get('start'))
        elif a == 'start':
            self.start()
        elif a == 'stop':
            self.stop('사용자 중지')
        elif a == 'clear':
            with self.lock:
                if self.phase not in ('TRACKING', 'WAIT_AUTO', 'REPLANNING', 'PLANNING'):
                    self.plan, self.goal = None, None
                    self._set('IDLE' if self.geom else 'CONFIG_INCOMPLETE', '경로 삭제')
                    self.send_paths()

    def _set(self, phase, msg):
        self.phase, self.message = phase, msg
        print(f'[autopark] {phase}: {msg}', flush=True)

    # ---------------- 계획
    def _start_state(self, manual):
        fb = self.feedback
        v = fb['values']
        if fb['planning_start_ok']:
            return (v['x'], v['y'], v['th'], v['beta']), False, '현재 위치(SLAM)+MT6701'
        if manual:
            beta = v.get('beta')
            note = '검증용 수동 시작자세' + ('' if beta is not None else ', 연결각 0 가정(MT6701 없음)')
            return (float(manual['x']), float(manual['y']), float(manual['yaw']),
                    float(beta) if beta is not None else 0.0), True, note
        return None, True, '시작 자세 없음: ' + '; '.join(fb['reasons'][:3]) + ' — 웹 UI에서 검증용 시작자세를 지정하세요'

    def request_plan(self, goal, manual_start=None, replan=False):
        with self.lock:
            if self.geom is None:
                return self._set('CONFIG_INCOMPLETE', '설정 누락: ' + ', '.join(self.missing))
            if self.planning:
                return
            if not replan and self.phase in ('TRACKING', 'WAIT_AUTO', 'REPLANNING'):
                return self._set(self.phase, '주행 중에는 새 계획 불가 — 먼저 중지')
            if self.grid is None:
                return self._set('PLAN_FAILED', 'SLAM 지도(/map) 미수신')
            if goal is None:
                return self._set('PLAN_FAILED', '목표 미지정')
            start, validation_only, note = self._start_state(None if replan else manual_start)
            if start is None:
                return self._set('PLAN_FAILED', note)
            self.goal = (float(goal['x']), float(goal['y']), float(goal['yaw']))
            geom = self._geom_capped()
            grid = self.grid
            self.planning = True
            self._set('REPLANNING' if replan else 'PLANNING', f'경로 계획 중 ({note})')

        def work():
            t0 = time.monotonic()
            try:
                res, traj, goal_state = plan_parking(self.cfg, geom, grid, start, self.goal)
            except Exception as e:
                res, traj, goal_state = None, None, None
                err = f'계획 예외: {e}'
            with self.lock:
                self.planning = False
                if res is None or not res.ok:
                    msg = err if res is None else f'Hybrid A* 실패 [{res.reason}] {res.message} {res.stats}'
                    self.plan = None
                    if replan:
                        self.stop('재계획 실패: ' + msg)
                    else:
                        self._set('PLAN_FAILED', msg)
                    self.send_paths()
                    return
                self.plan_counter += 1
                self.plan = {'id': self.plan_counter, 'astar': res, 'traj': traj, 'goal_state': goal_state,
                             'start': start, 'validation_only': validation_only, 'geom': geom, 'grid': grid,
                             'time_s': round(time.monotonic() - t0, 2), 'note': note}
                self.send_paths()
                if replan and self.phase == 'REPLANNING':
                    self.tracker = TrackingMPC(geom, self.cfg)
                    self.tracker.reset(traj, self._steer_now())
                    self._set('TRACKING', f'재계획 {self.replans}회 후 추종 재개')
                else:
                    extra = ' — 검증용(실제 구동 불가)' if validation_only else ''
                    self._set('PLANNED', f'계획 완료 {traj.summary()["length_m"]} m, '
                                         f'{len(traj.segments)}구간, MPC {"OK" if traj.mpc_ok else "실패→A*"}{extra}')
        threading.Thread(target=work, daemon=True).start()

    def _geom_capped(self):
        """Pi 가 보고하는 현재 조향 한계가 더 작으면 그 값 사용."""
        g = self.geom
        steer = ((self.state or {}).get('vehicle') or {}).get('steering') or {}
        lim = [steer.get('limit_left_rad'), steer.get('limit_right_rad')]
        lim = [x for x in lim if isinstance(x, (int, float)) and x > 0]
        if lim and min(lim) < g.steer_max:
            from dataclasses import replace
            self.notes = [f'Pi 조향 한계 {min(lim):.3f} rad 로 제한']
            return replace(g, steer_max=min(lim))
        return g

    def _steer_now(self):
        st = ((self.state or {}).get('vehicle') or {}).get('steering') or {}
        return st.get('cmd_rad') or 0.0

    # ---------------- 시작/정지
    def start(self):
        with self.lock:
            fb = self.feedback
            if self.phase != 'PLANNED' or self.plan is None:
                return self._set(self.phase, '시작 불가: 계획된 경로 없음')
            if self.plan['validation_only']:
                return self._set('PLANNED', '시작 불가: 검증용(수동 시작자세) 경로 — 실제 위치로 다시 계획 필요')
            if not fb['motion_ok']:
                return self._set('PLANNED', '시작 차단: ' + '; '.join(fb['reasons']))
            v, s0 = fb['values'], self.plan['start']
            f = self.cfg['feedback']
            if math.hypot(v['x'] - s0[0], v['y'] - s0[1]) > f['start_tol_xy_m'] or \
                    abs(wrap(v['th'] - s0[2])) > f['start_tol_yaw_rad']:
                return self._set('PLANNED', '시작 차단: 계획 후 차량이 움직임 → 다시 계획')
            self.replans = 0
            self.tracker = TrackingMPC(self.plan['geom'], self.cfg)
            self.wait_t = time.monotonic()
            self._set('WAIT_AUTO', 'AUTO_PARK 전환 요청 (Pi 가 정지 확인 후 전환)')
        self.link.send({'type': 'ping', 't': time.time(), 'ready': True})
        self.link.send({'type': 'mode', 'mode': 'AUTO_PARK'})

    def stop(self, reason):
        with self.lock:
            active = self.phase in ('WAIT_AUTO', 'TRACKING', 'REPLANNING')
            self._set('STOPPED', reason)
        try:
            self.link.send({'type': 'stop'})
            self.link.send({'type': 'ping', 't': time.time(), 'ready': False})
            if active:
                self.link.send({'type': 'mode', 'mode': 'MANUAL'})
        except OSError:
            pass

    # ---------------- 20 Hz
    def tick(self):
        now = time.monotonic()
        with self.lock:
            self.feedback = self.mon.evaluate(self.state, self.state_t, now)
            fb, phase = self.feedback, self.phase
            ctrl = (self.state or {}).get('control') or {}
            mode = ctrl.get('mode')
        ready = phase in ('WAIT_AUTO', 'TRACKING', 'REPLANNING') and fb['motion_ok']
        if phase == 'WAIT_AUTO':
            if mode == 'AUTO_PARK' and not ctrl.get('pending_mode'):
                with self.lock:
                    self.tracker.reset(self.plan['traj'], self._steer_now())
                    self._set('TRACKING', '경로 추종 시작')
            elif now - self.wait_t > 6.0:
                self.stop('AUTO_PARK 전환 시간 초과')
            elif not fb['motion_ok']:
                self.stop('피드백 무효: ' + '; '.join(fb['reasons']))
            return ready
        if phase in ('TRACKING', 'REPLANNING'):
            if mode != 'AUTO_PARK' or ctrl.get('pending_mode') == 'MANUAL':
                evs = ctrl.get('events') or [{}]
                self.stop(f'Pi 가 AUTO_PARK 해제: {evs[-1].get("msg", "")}')
                return False
            if not fb['motion_ok']:
                self.stop('피드백 손실 → 안전정지: ' + '; '.join(fb['reasons']))
                return False
            if phase == 'REPLANNING':
                self.link.send({'type': 'drive', 'v': 0.0, 'steer': float(self._steer_now())})
                return ready
            v = fb['values']
            out = self.tracker.step({'x': v['x'], 'y': v['y'], 'th': v['th'], 'beta': v['beta'],
                                     'v': v['v'], 'us_min': v.get('us_min'), 't': now})
            self.track_out = out
            if out.status in ('TRACKING', 'SEGMENT_STOP'):
                self.link.send({'type': 'drive', 'v': float(out.v), 'steer': float(out.steer)})
            elif out.status == 'PARKED':
                self.link.send({'type': 'stop'})
                self.link.send({'type': 'mode', 'mode': 'MANUAL'})
                with self.lock:
                    self._set('PARKED', f'주차 완료 {out.info.get("goal_error")}')
            elif out.status == 'REPLAN':
                self.link.send({'type': 'drive', 'v': 0.0, 'steer': float(out.steer or self._steer_now())})
                with self.lock:
                    self.replans += 1
                    over = self.replans > self.cfg['tracking_mpc']['max_replans']
                if over:
                    self.stop(f'재계획 한도 초과 ({out.reason})')
                else:
                    self.request_plan({'x': self.goal[0], 'y': self.goal[1], 'yaw': self.goal[2]}, replan=True)
            else:
                self.stop(f'안전정지: {out.reason}')
        return ready

    # ---------------- UI 송신
    def status_msg(self):
        with self.lock:
            p = self.plan
            o = self.track_out if self.phase in ('TRACKING', 'REPLANNING', 'PARKED', 'STOPPED') else None
            return {
                'type': 'park_status', 'phase': self.phase, 'message': self.message,
                'motion_ok': self.feedback['motion_ok'], 'block_reasons': self.feedback['reasons'],
                'feedback': self.feedback['checks'],
                'config_missing': self.missing,
                'plan': None if p is None else {
                    'id': p['id'], 'validation_only': p['validation_only'], 'time_s': p['time_s'],
                    'note': p['note'], 'astar': p['astar'].stats, **p['traj'].summary()},
                'tracking': None if o is None else {
                    'status': o.status, 'reason': o.reason, 'v_cmd': round(o.v, 3), 'steer_cmd': round(o.steer, 4),
                    'errors': o.errors, 'info': {k: v for k, v in o.info.items() if k != 'predicted'},
                    'predicted': o.info.get('predicted')},
                'replans': self.replans, 'notes': self.notes,
            }

    def send_paths(self):
        p, g = self.plan, self.geom
        msg = {'type': 'park_paths', 'goal': None, 'geometry': g.to_dict() if g else None}
        if self.goal is not None:
            msg['goal'] = {'x': self.goal[0], 'y': self.goal[1], 'yaw': self.goal[2]}
        if p is not None:
            fp = Footprint(p['geom'])
            ast = p['astar'].states
            tj = p['traj']
            k = max(1, len(ast) // 400)
            msg['astar'] = [round(c, 3) for s in ast[::k] for c in s[:2]]
            msg['mpc'] = [round(float(c), 3) for s in tj.states for c in s[:2]]
            msg['mpc_trailer'] = [round(c, 3) for s in tj.states for c in trailer_pose(tuple(s), p['geom'])[:2]]
            msg['segments'] = tj.segments
            msg['goal_outline'] = fp.outline(p['goal_state'])
            msg['start_outline'] = fp.outline(p['start'])
            msg['plan_id'] = p['id']
            msg['validation_only'] = p['validation_only']
        try:
            self.link.send(msg)
        except OSError:
            pass


def run(args, cfg):
    while True:
        try:
            ws = WS(args.host, args.port)
        except OSError as e:
            print(f'접속 실패 {args.host}:{args.port} ({e}) — 2초 후 재시도', flush=True)
            time.sleep(2)
            continue
        print(f'접속: ws://{args.host}:{args.port}/ws', flush=True)
        ctl = AutoParkController(cfg, ws)
        ws.send({'type': 'hello', 'role': 'autopark', 'name': socket.gethostname(), 'ready': False})
        alive = [True]

        def loop():
            n = 0
            while alive[0]:
                try:
                    ready = ctl.tick()
                    if n % 10 == 0:
                        ws.send({'type': 'ping', 't': time.time(), 'ready': bool(ready)})
                    if n % 4 == 0:
                        ws.send(ctl.status_msg())
                except OSError:
                    break
                except Exception as e:  # 알고리즘 예외 → 안전정지
                    print(f'[autopark] 제어 루프 예외: {e!r}', flush=True)
                    ctl.stop(f'내부 오류: {e!r}')
                n += 1
                time.sleep(0.05)
        th = threading.Thread(target=loop, daemon=True)
        th.start()
        try:
            while True:
                m = ws.recv()
                t = m.get('type')
                if t == 'state':
                    ctl.on_state(m)
                elif t == 'map':
                    ctl.on_map(m)
                elif t == 'park_cmd':
                    ctl.on_park_cmd(m)
                elif t == 'reply':
                    ctl.on_reply(m)
                elif t == 'welcome':
                    pc = m.get('config') or {}
                    wb = pc.get('wheelbase_m')
                    if wb is not None and abs(wb - cfg['vehicle']['wheelbase_m']) > 1e-6:
                        print(f'[경고] 휠베이스 불일치: Pi {wb} vs autopark {cfg["vehicle"]["wheelbase_m"]}', flush=True)
                    ctl.send_paths()
        except (OSError, ConnectionError, ValueError) as e:
            print(f'연결 끊김: {e} (차량은 명령 타임아웃으로 자동 정지)', flush=True)
        except KeyboardInterrupt:
            ctl.stop('노트북 프로그램 종료')
            alive[0] = False
            ws.close()
            return
        alive[0] = False
        ws.close()
        time.sleep(1)


# ====================================================================== 오프라인 계획 (지도 파일)
def write_svg(path, grid, res, traj, start, goal_state, geom):
    fp = Footprint(geom)
    pts = [s[:2] for s in res.states] + [start[:2], goal_state[:2]]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    x0, x1, y0, y1 = min(xs) - 1, max(xs) + 1, min(ys) - 1, max(ys) + 1
    sc = 200.0
    W, H = (x1 - x0) * sc, (y1 - y0) * sc

    def P(x, y):
        return f'{(x - x0) * sc:.1f},{(y1 - y) * sc:.1f}'
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{H:.0f}" style="background:#fff">']
    r = grid.res
    c0, c1 = max(0, int((x0 - grid.ox) / r)), min(grid.w, int((x1 - grid.ox) / r) + 1)
    r0, r1 = max(0, int((y0 - grid.oy) / r)), min(grid.h, int((y1 - grid.oy) / r) + 1)
    for row in range(r0, r1):
        for col in range(c0, c1):
            if grid.occ[row, col]:
                x, y = grid.ox + col * r, grid.oy + (row + 1) * r
                out.append(f'<rect x="{(x - x0) * sc:.1f}" y="{(y1 - y) * sc:.1f}" width="{r * sc:.1f}" '
                           f'height="{r * sc:.1f}" fill="#333"/>')
    out.append('<polyline fill="none" stroke="#f0883e" stroke-width="2" points="' +
               ' '.join(P(s[0], s[1]) for s in res.states) + '"/>')
    if traj is not None:
        out.append('<polyline fill="none" stroke="#1f6feb" stroke-width="3" points="' +
                   ' '.join(P(s[0], s[1]) for s in traj.states) + '"/>')
        out.append('<polyline fill="none" stroke="#a371f7" stroke-width="2" stroke-dasharray="4 3" points="' +
                   ' '.join(P(*trailer_pose(tuple(s), geom)[:2]) for s in traj.states) + '"/>')
    for st, col in ((start, '#2da44e'), (goal_state, '#cf222e')):
        for rect in fp.outline(st):
            out.append(f'<polygon fill="none" stroke="{col}" stroke-width="2" points="' +
                       ' '.join(P(*p) for p in rect) + '"/>')
    out.append('</svg>')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(out))


def plan_cmd(args, cfg):
    geom, missing = geometry_from_config(cfg)
    if geom is None:
        print('설정 누락 → 계획 불가:', ', '.join(missing))
        return 2
    if args.map:
        grid = GridMap.from_pgm_yaml(args.map, cfg['map'])
    else:
        w, h = (float(v) for v in args.empty.lower().split('x'))
        grid = GridMap.empty(w, h)
    start = tuple(args.start)
    t0 = time.monotonic()
    res, traj, goal_state = plan_parking(cfg, geom, grid, start, tuple(args.goal))
    print(f'Hybrid A*: ok={res.ok} reason={res.reason} {res.message} stats={res.stats}')
    if res.ok:
        print('Planning MPC:', traj.summary())
        tx, ty, th2 = trailer_pose(tuple(traj.states[-1]), geom)
        print(f'최종 트레일러 자세: ({tx:.3f}, {ty:.3f}, {math.degrees(th2):.1f}°), '
              f'연결각 {math.degrees(traj.states[-1][3]):.1f}°  /  총 {time.monotonic() - t0:.1f} s')
        if args.svg:
            write_svg(args.svg, grid, res, traj, start, goal_state, geom)
            print('SVG 저장:', args.svg)
    return 0 if res.ok else 1


def main(argv=None):
    ap = argparse.ArgumentParser(prog='python -m autopark')
    ap.add_argument('--config', default=None)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run', help='Pi 에 접속하여 자동주차 클라이언트 실행')
    r.add_argument('host')
    r.add_argument('--port', type=int, default=8080)
    p = sub.add_parser('plan', help='지도 파일에서 경로 계획만 (구동 없음)')
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument('--map', help='map_server 형식 .yaml (Pi maps/ 에서 복사)')
    g.add_argument('--empty', help='장애물 없는 공간 WxH [m], 예: 6x4')
    p.add_argument('--start', type=float, nargs=4, required=True, metavar=('X', 'Y', 'YAW', 'BETA'))
    p.add_argument('--goal', type=float, nargs=3, required=True, metavar=('X', 'Y', 'YAW'),
                   help='목표 트레일러 차축 자세')
    p.add_argument('--svg', default=None)
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    if a.cmd == 'run':
        run(a, cfg)
        return 0
    return plan_cmd(a, cfg)


if __name__ == '__main__':
    sys.exit(main())
