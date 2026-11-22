"""공통 유틸: 설정 로드, pigpiod 소켓 클라이언트(외부 패키지 불필요), 쿼드러처 디코딩 테이블.

pigpiod 는 호스트(Raspberry Pi OS)에서 실행되고, Docker(network_mode: host) 내부 노드는
127.0.0.1:8888 소켓으로 접근한다. 프로토콜: 16바이트 명령(cmd,p1,p2,p3) + 확장데이터 → 16바이트 응답.
"""
import json
import math
import os
import socket
import struct
import threading
import time

import yaml

DEFAULT_CONFIG = '/rc_lidar_docker/config/vehicle.yaml'


def load_config(path=None):
    path = path or os.environ.get('RC_VEHICLE_CONFIG', DEFAULT_CONFIG)
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def dumps(obj):
    return json.dumps(obj, ensure_ascii=False, separators=(',', ':'), allow_nan=False,
                      default=_json_default)


def _json_default(o):
    return str(o)


def finite_or_none(x, nd=4):
    """JSON 에 NaN/Inf 를 넣지 않기 위해: 유효하지 않은 값은 None(null)."""
    if x is None:
        return None
    try:
        xf = float(x)
    except (TypeError, ValueError):
        return None
    if math.isnan(xf) or math.isinf(xf):
        return None
    return round(xf, nd)


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def tick_diff(t_new, t_old):
    """pigpio tick(us, uint32, 약 72분마다 wrap) 차이."""
    return (t_new - t_old) & 0xFFFFFFFF


# 쿼드러처 디코딩: index = (prev_state << 2) | new_state, state = (A << 1) | B
# 0 = 변화없음/무효(두 상 동시 변화 → 에러 카운트)
QDEC = (0, +1, -1, 0,
        -1, 0, 0, +1,
        +1, 0, 0, -1,
        0, -1, +1, 0)
QDEC_INVALID = (3, 6, 9, 12)


class PigpioError(Exception):
    pass


class Pigpio:
    """pigpiod 최소 클라이언트 (필요한 명령만 구현)."""
    MODES, PUD, READ, WRITE, PWM, PRS, PFS, SERVO = 0, 2, 3, 4, 5, 6, 7, 8
    TICK, HWVER, NB, NC, PIGPV, TRIG = 16, 17, 19, 21, 26, 37
    PROC, PROCD, PROCR, PROCS, NOIB, PROCU = 38, 39, 40, 41, 99, 117
    INPUT, OUTPUT = 0, 1
    PUD_MAP = {'off': 0, 'down': 1, 'up': 2}

    def __init__(self, host='127.0.0.1', port=8888, timeout=2.0):
        self.host, self.port = host, port
        self.s = socket.create_connection((host, port), timeout=timeout)
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.lock = threading.Lock()

    @staticmethod
    def _recv_exact(sock, n):
        buf = b''
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise PigpioError('pigpiod 연결 끊김')
            buf += chunk
        return buf

    def cmd(self, cmd, p1=0, p2=0, ext=b'', check=True):
        with self.lock:
            self.s.sendall(struct.pack('<IIII', cmd, p1 & 0xFFFFFFFF, p2 & 0xFFFFFFFF, len(ext)) + ext)
            res = struct.unpack('<12xi', self._recv_exact(self.s, 16))[0]
        if check and res < 0:
            raise PigpioError(f'pigpio 명령 {cmd}(p1={p1},p2={p2}) 오류코드 {res}')
        return res

    # --- GPIO ---
    def set_mode(self, g, mode):
        self.cmd(self.MODES, g, mode)

    def set_pull(self, g, pull):
        self.cmd(self.PUD, g, self.PUD_MAP[str(pull).lower()])

    def read(self, g):
        return self.cmd(self.READ, g)

    def write(self, g, level):
        self.cmd(self.WRITE, g, 1 if level else 0)

    def set_pwm_frequency(self, g, f):
        return self.cmd(self.PFS, g, int(f))

    def set_pwm_range(self, g, r):
        return self.cmd(self.PRS, g, int(r))

    def set_pwm(self, g, duty):
        self.cmd(self.PWM, g, int(duty))

    def set_servo(self, g, pulse_us):
        self.cmd(self.SERVO, g, int(pulse_us))

    def trigger(self, g, pulse_us, level=1):
        self.cmd(self.TRIG, g, int(pulse_us), struct.pack('<I', level))

    def version(self):
        return self.cmd(self.PIGPV)

    def hw_version(self):
        return self.cmd(self.HWVER)

    # --- 스크립트 (워치독) ---
    def store_script(self, text):
        return self.cmd(self.PROC, 0, 0, text.encode('ascii'))

    def run_script(self, sid, params=()):
        ext = struct.pack('<' + 'I' * len(params), *params) if params else b''
        return self.cmd(self.PROCR, sid, 0, ext)

    def update_script(self, sid, params):
        ext = struct.pack('<' + 'I' * len(params), *params)
        return self.cmd(self.PROCU, sid, 0, ext)

    def stop_script(self, sid):
        return self.cmd(self.PROCS, sid, check=False)

    def delete_script(self, sid):
        return self.cmd(self.PROCD, sid, check=False)

    def close(self):
        try:
            self.s.close()
        except OSError:
            pass


class NotifyStream:
    """pigpiod in-band 알림 스트림. 감시 비트마스크의 GPIO 레벨이 바뀔 때마다 12바이트 레코드
    (seq:u16, flags:u16, tick:u32[us], level:u32[GPIO0-31 비트마스크]) 를 받는다.
    pigpiod 가 5us 간격으로 샘플링하므로 엔코더 엣지 타이밍을 커널 지터 없이 얻을 수 있다."""
    REC = struct.Struct('<HHII')
    FLAG_EVENT, FLAG_ALIVE, FLAG_WDOG = 1 << 7, 1 << 6, 1 << 5

    def __init__(self, ctrl: Pigpio, bits):
        self.ctrl = ctrl
        self.s = socket.create_connection((ctrl.host, ctrl.port), timeout=5.0)
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.s.sendall(struct.pack('<IIII', Pigpio.NOIB, 0, 0, 0))
        self.handle = struct.unpack('<12xi', Pigpio._recv_exact(self.s, 16))[0]
        if self.handle < 0:
            raise PigpioError(f'알림 핸들 열기 실패 {self.handle}')
        self.s.settimeout(None)
        ctrl.cmd(Pigpio.NB, self.handle, bits)
        self._rest = b''

    def read_records(self):
        """블로킹 수신 후 (flags, tick, level) 리스트 반환. 연결 끊기면 PigpioError."""
        data = self.s.recv(65536)
        if not data:
            raise PigpioError('pigpiod 알림 스트림 끊김')
        data = self._rest + data
        n = len(data) - (len(data) % 12)
        self._rest = data[n:]
        return [(f, t, lv) for (_seq, f, t, lv) in self.REC.iter_unpack(data[:n])]

    def close(self):
        try:
            self.ctrl.cmd(Pigpio.NC, self.handle, check=False)
        except Exception:
            pass
        try:
            self.s.close()
        except OSError:
            pass


def watchdog_script(motor_pins, timeout_ms):
    """pigpiod 내부에서 도는 스크립트: timeout_ms 동안 p0 갱신(=1)이 없으면 모터 핀을 LOW(=MDD3A 브레이크).
    노드가 SIGKILL/크래시로 죽어도 pigpiod 가 마지막 PWM을 계속 내보내는 위험을 막는다."""
    writes = ' '.join(f'w {g} 0' for g in motor_pins)
    return (f'tag 100 mils {int(timeout_ms)} lda p0 jz 200 ld p0 0 jmp 100 '
            f'tag 200 {writes} jmp 100')


def now():
    return time.monotonic()
