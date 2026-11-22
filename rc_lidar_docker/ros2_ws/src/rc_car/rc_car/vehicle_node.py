"""공통 차량 제어부 (Vehicle Control) — 하드웨어를 직접 다루는 유일한 노드.

  입력  /rc/drive_cmd   (std_msgs/String JSON, Command Manager 가 20 Hz 발행 — 최종 명령)
        /rc/settings    (std_msgs/String JSON, 출력 제한 변경)
        /rc/hw_request  (std_msgs/String JSON, trailer_zero / fault_reset)
  출력  /joint_states        sensor_msgs/JointState  (뒷바퀴 2개 위치[rad]/속도[rad/s], trailer_joint)
        /steering/angle_cmd  std_msgs/Float64        (서보 명령 조향각 rad — 측정값 아님)
        /trailer/angle       std_msgs/Float64        (MT6701 트레일러 상대각 rad, ppr 설정 시에만)
        /ultrasonic/<name>   sensor_msgs/Range       (실패 시 range = NaN)
        /rc/vehicle_status   std_msgs/String JSON    (10 Hz, 웹/Command Manager 용 전체 상태)

하드웨어 접근: 호스트 pigpiod (DMA PWM, 서보 펄스, 5us 엣지 샘플링 알림 스트림).
"""
import collections
import json
import math
import os
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Float64, String

from .common import (QDEC, NotifyStream, Pigpio, PigpioError, clamp, dumps, finite_or_none,
                     load_config, tick_diff, watchdog_script, wrap_pi)


class QuadEncoder:
    def __init__(self, gpio_a, gpio_b, invert=False):
        self.a, self.b = gpio_a, gpio_b
        self.mask = (1 << gpio_a) | (1 << gpio_b)
        self.sign = -1 if invert else 1
        self.state = 0
        self.count = 0
        self.errors = 0
        self.last_tick = 0
        self.edges = 0

    def init(self, level):
        self.state = (((level >> self.a) & 1) << 1) | ((level >> self.b) & 1)

    def update(self, level, tick):
        s = (((level >> self.a) & 1) << 1) | ((level >> self.b) & 1)
        if s != self.state:
            d = QDEC[(self.state << 2) | s]
            if d == 0:
                self.errors += 1          # 두 상이 동시에 바뀜 → 엣지 누락(샘플링 한계/노이즈)
            else:
                self.count += d * self.sign
                self.last_tick = tick
                self.edges += 1
            self.state = s


class Motor:
    def __init__(self, cfg, cpr):
        self.cfg = cfg
        self.name = cfg['name']
        self.enc = QuadEncoder(cfg['enc_a'], cfg['enc_b'], cfg.get('invert_encoder', False))
        self.cpr = cpr
        self.samples = collections.deque(maxlen=64)   # (host_t, count, last_tick)
        self.target_rpm = 0.0      # 램프 적용 후 목표
        self.measured_rpm = 0.0
        self.duty = 0.0            # -1..1 (+ = 전진 방향)
        self.integ = 0.0
        self.applied = (None, None)
        self.last_count = 0
        self.last_move_t = time.monotonic()
        self.reverse_since = None
        self.net_m = 0.0


class VehicleNode(Node):
    def __init__(self):
        super().__init__('rc_vehicle')
        self.declare_parameter('config_file', '')
        self.cfg = load_config(self.get_parameter('config_file').value or None)
        c = self.cfg
        m = c['motors']
        self.cpr = m['encoder_ppr'] * m['quadrature_multiplier'] * m['gear_ratio']
        # 사람이 엔코더 동작을 확인(README §6-3·6-4)하기 전에는 엔코더 값을 피드백으로 쓰지 않는다.
        # (신호가 없어 카운트가 0 으로 고정돼도 '정지 중'과 구분할 수 없기 때문)
        self.enc_verified = bool(m.get('encoders_verified', False))
        self.wheel_d = c['vehicle']['wheel_diameter_m']
        self.motors = [Motor(m['left'], self.cpr), Motor(m['right'], self.cpr)]
        sc = c['speed_control']
        self.dt_nominal = 1.0 / sc['loop_hz']

        # ---- 출력 상한 (데이터시트 기반 하드 리밋 — 런타임 변경 불가) ----
        p = c['power']
        self.motors_disabled_reason = None
        vb = p.get('battery_voltage_max')
        if vb is None:
            self.hard_max_pwm = 0.0
            self.motors_disabled_reason = 'config power.battery_voltage_max 미설정 → 모터 출력 비활성화'
        elif not (p['driver_vin_min'] <= vb <= p['driver_vin_max']):
            self.hard_max_pwm = 0.0
            self.motors_disabled_reason = f'배터리 {vb} V 가 MDD3A 허용 {p["driver_vin_min"]}–{p["driver_vin_max"]} V 밖'
        else:
            self.hard_max_pwm = min(1.0, p['motor_rated_voltage'] / vb)
        self.ff_scale = (p['motor_rated_voltage'] / vb) if vb else 0.0

        # ---- 조향 한계 ----
        st = c['steering']
        self.st = st
        pmin = max(st['pulse_min_us'], st['pulse_abs_min_us'])
        pmax = min(st['pulse_max_us'], st['pulse_abs_max_us'])
        self.pulse_lim = (pmin, pmax)
        k = st['us_per_rad']
        span_hi, span_lo = (pmax - st['center_us']) / k, (st['center_us'] - pmin) / k
        # +각 = 좌회전(ROS 규약)
        self.steer_lim_left, self.steer_lim_right = (span_lo, span_hi) if st.get('invert') else (span_hi, span_lo)
        self.steer_hw_limit = max(0.0, min(self.steer_lim_left, self.steer_lim_right))

        # ---- 런타임 설정 (웹 UI 조절) ----
        self.settings = {
            'max_rpm': float(clamp(sc['max_rpm_default'], 0, m['rated_rpm'])),
            'max_pwm': float(clamp(sc['max_pwm_default'], 0, self.hard_max_pwm)),
            'max_steer_rad': self.steer_hw_limit,
        }

        # ---- 트레일러 (MT6701 ABZ) ----
        tr = c['trailer']
        self.tr = tr
        self.trailer_enc = QuadEncoder(tr['enc_a'], tr['enc_b'], tr.get('invert', False))
        self.trailer_cpr = (tr['ppr'] * 4) if tr.get('ppr') else None
        self.trailer_zero_count = 0
        self.trailer_zero_source = 'boot'
        self.z_count = None          # 최근 Z 펄스 시점 카운트
        self.z_period = None         # 연속 Z 사이 카운트 (= 1회전 카운트 측정)
        self.z_prev_level = 0
        self.saved_z_offset = self._load_trailer_zero()

        # ---- 초음파 ----
        us = c['ultrasonic']
        self.us_cfg = us
        self.us = [{'cfg': s, 'range': None, 'status': 'NO_DATA', 't': 0.0, 'fails': 0} for s in us['sensors']]
        self.echo_pins = {s['echo']: i for i, s in enumerate(us['sensors'])}
        self.echo_rise = {}
        self.echo_width = {}
        self.echo_event = threading.Event()
        self.echo_waiting = None

        # ---- 상태 ----
        self.lock = threading.Lock()
        self.pi = None
        self.notify = None
        self.notify_alive = False
        self.notify_rate = 0.0
        self.records = 0
        self.wd_sid = None
        self.wd_ok = not c['pigpio'].get('watchdog_script', True)
        self.cmd = None
        self.cmd_rx = 0.0
        self.steer_target = 0.0
        self.steer_cmd = 0.0
        self.steer_pulse = None
        self.latched_faults = []
        self.warnings = []
        self.odometer = 0.0
        self.last_step = time.monotonic()
        self.last_status = 0.0
        self.last_reconnect = 0.0
        self.running = True
        self.last_level = 0
        self.active_mode, self.stop_reason, self.cmd_age = 'stop', 'NO_COMMAND', None

        # ---- ROS I/O ----
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_steer = self.create_publisher(Float64, '/steering/angle_cmd', 10)
        self.pub_trailer = self.create_publisher(Float64, '/trailer/angle', 10)
        self.pub_status = self.create_publisher(String, '/rc/vehicle_status', 10)
        self.pub_us = [self.create_publisher(Range, f'/ultrasonic/{s["name"]}', 10) for s in us['sensors']]
        self.create_subscription(String, '/rc/drive_cmd', self._on_cmd, 10)
        self.create_subscription(String, '/rc/settings', self._on_settings, 10)
        self.create_subscription(String, '/rc/hw_request', self._on_hw_request, 10)

        self._connect()
        self.create_timer(self.dt_nominal, self._step)
        threading.Thread(target=self._ultrasonic_loop, daemon=True).start()

    # ================================================================ 연결/초기화
    def _connect(self):
        self.last_reconnect = time.monotonic()
        pc = self.cfg['pigpio']
        try:
            pi = Pigpio(pc['host'], pc['port'])
            self.get_logger().info(f'pigpiod v{pi.version()} 연결 (HW rev {pi.hw_version():x})')
            m = self.cfg['motors']
            motor_pins = []
            for mo in self.motors:
                for key in ('pwm_a', 'pwm_b'):
                    g = mo.cfg[key]
                    motor_pins.append(g)
                    pi.set_mode(g, Pigpio.OUTPUT)
                    pi.write(g, 0)
                    pi.set_pwm_frequency(g, m['pwm_frequency_hz'])
                    pi.set_pwm_range(g, m['pwm_range'])
                mo.applied = (None, None)
                for key in ('enc_a', 'enc_b'):
                    pi.set_mode(mo.cfg[key], Pigpio.INPUT)
                    pi.set_pull(mo.cfg[key], m.get('encoder_pull', 'up'))
            for key in ('enc_a', 'enc_b', 'enc_z'):
                pi.set_mode(self.tr[key], Pigpio.INPUT)
                pi.set_pull(self.tr[key], self.tr.get('pull', 'off'))
            for s in self.us_cfg['sensors']:
                pi.set_mode(s['trig'], Pigpio.OUTPUT)
                pi.write(s['trig'], 0)
                pi.set_mode(s['echo'], Pigpio.INPUT)
                pi.set_pull(s['echo'], 'off')
            pi.set_mode(self.st['gpio'], Pigpio.OUTPUT)
            self.steer_pulse = None

            # 워치독 스크립트 (pigpiod 내부)
            if pc.get('watchdog_script', True):
                self.wd_ok = False
                for sid in range(32):          # 이전 실행이 남긴 스크립트 정리 (pigpiod 는 이 프로젝트 전용 전제)
                    pi.stop_script(sid)
                    pi.delete_script(sid)
                sid = pi.store_script(watchdog_script(motor_pins, pc.get('watchdog_timeout_ms', 300)))
                for _ in range(20):
                    try:
                        pi.run_script(sid, (1,))
                        self.wd_sid, self.wd_ok = sid, True
                        break
                    except PigpioError:
                        time.sleep(0.05)
                if not self.wd_ok:
                    self.get_logger().error('pigpio 워치독 스크립트 실행 실패 → 모터 비활성화 '
                                            '(config pigpio.watchdog_script: false 로 끌 수 있음)')

            # 알림 스트림: 엔코더 + MT6701 + 초음파 에코
            bits = 0
            for mo in self.motors:
                bits |= mo.enc.mask
            bits |= self.trailer_enc.mask | (1 << self.tr['enc_z'])
            for g in self.echo_pins:
                bits |= 1 << g
            level = pi.cmd(10, check=False) & 0xFFFFFFFF   # BR1: GPIO0-31 레벨
            with self.lock:
                for mo in self.motors:
                    mo.enc.init(level)
                self.trailer_enc.init(level)
                self.z_prev_level = (level >> self.tr['enc_z']) & 1
                self.last_level = level
            self.notify = NotifyStream(pi, bits)
            self.pi = pi
            self.notify_alive = True
            threading.Thread(target=self._notify_loop, args=(self.notify,), daemon=True).start()
        except (OSError, PigpioError) as e:
            self.get_logger().error(f'pigpiod 연결/초기화 실패: {e} — 호스트에서 pigpiod 실행 여부 확인. 2초 후 재시도')
            self.pi = None
            self.notify_alive = False

    def _drop_connection(self, why):
        if self.pi is not None and why != '종료':
            self.get_logger().error(f'pigpiod 연결 문제: {why}')
        self.notify_alive = False
        if self.pi is not None:
            try:   # 소켓이 살아 있으면 즉시 브레이크 (죽었으면 pigpiod 워치독 스크립트가 처리)
                for mo in self.motors:
                    self.pi.write(mo.cfg['pwm_a'], 0)
                    self.pi.write(mo.cfg['pwm_b'], 0)
                    mo.duty, mo.integ, mo.target_rpm = 0.0, 0.0, 0.0
            except Exception:
                pass
        try:
            if self.notify:
                self.notify.close()
        except Exception:
            pass
        if self.pi:
            self.pi.close()
        self.pi = None
        self.notify = None

    # ================================================================ 엣지 처리 스레드
    def _notify_loop(self, stream):
        enc = [mo.enc for mo in self.motors] + [self.trailer_enc]
        zbit = 1 << self.tr['enc_z']
        echo_bits = {g: 1 << g for g in self.echo_pins}
        last_rate_t, last_rate_n = time.monotonic(), 0
        while self.running and stream is self.notify:
            try:
                recs = stream.read_records()
            except (OSError, PigpioError) as e:
                if stream is self.notify:
                    self.notify_alive = False
                    self.get_logger().error(f'알림 스트림 종료: {e}')
                return
            with self.lock:
                last = self.last_level
                for flags, tick, level in recs:
                    if flags:
                        continue
                    changed = level ^ last
                    last = level
                    for e in enc:
                        if changed & e.mask:
                            e.update(level, tick)
                    if changed & zbit and (level & zbit):
                        self._on_z(tick)
                    for g, b in echo_bits.items():
                        if changed & b:
                            if level & b:
                                self.echo_rise[g] = tick
                            elif g in self.echo_rise:
                                self.echo_width[g] = tick_diff(tick, self.echo_rise.pop(g))
                                if self.echo_waiting == g:
                                    self.echo_event.set()
                self.last_level = last
                self.records += len(recs)
            t = time.monotonic()
            if t - last_rate_t >= 1.0:
                self.notify_rate = (self.records - last_rate_n) / (t - last_rate_t)
                last_rate_t, last_rate_n = t, self.records

    def _on_z(self, tick):
        c = self.trailer_enc.count
        if self.z_count is not None and c != self.z_count:
            self.z_period = abs(c - self.z_count)
        self.z_count = c
        if self.trailer_zero_source == 'boot' and self.saved_z_offset is not None:
            self.trailer_zero_count = c + self.saved_z_offset
            self.trailer_zero_source = 'z_index'

    # ================================================================ 구독 콜백
    def _on_cmd(self, msg):
        try:
            self.cmd = json.loads(msg.data)
            self.cmd_rx = time.monotonic()
        except ValueError:
            pass

    def _on_settings(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        s = self.settings
        if 'max_rpm' in d:
            s['max_rpm'] = float(clamp(float(d['max_rpm']), 0.0, self.cfg['motors']['rated_rpm']))
        if 'max_pwm' in d:
            s['max_pwm'] = float(clamp(float(d['max_pwm']), 0.0, self.hard_max_pwm))
        if 'max_steer_rad' in d:
            s['max_steer_rad'] = float(clamp(float(d['max_steer_rad']), 0.0, self.steer_hw_limit))
        self.get_logger().info(f'설정 변경: {s}')

    def _on_hw_request(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        t = d.get('type')
        if t == 'fault_reset':
            if self.latched_faults:
                self.get_logger().warn(f'고장 래치 해제: {self.latched_faults}')
            self.latched_faults = []
        elif t == 'trailer_zero':
            with self.lock:
                self.trailer_zero_count = self.trailer_enc.count
                self.trailer_zero_source = 'user'
                if self.z_count is not None:
                    self.saved_z_offset = self.trailer_zero_count - self.z_count
                    self._save_trailer_zero(self.saved_z_offset)

    def _load_trailer_zero(self):
        try:
            with open(self.tr['zero_file'], 'r', encoding='utf-8') as f:
                return int(json.load(f)['offset_from_z'])
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def _save_trailer_zero(self, off):
        try:
            with open(self.tr['zero_file'], 'w', encoding='utf-8') as f:
                json.dump({'offset_from_z': off, 'ppr': self.tr.get('ppr'), 'saved': time.time()}, f)
        except OSError as e:
            self.get_logger().warn(f'트레일러 영점 저장 실패: {e}')

    # ================================================================ 제어 루프 (50 Hz)
    def _latch(self, code):
        if code not in self.latched_faults:
            self.latched_faults.append(code)
            self.get_logger().error(f'고장 래치: {code} → 정지 (웹 UI에서 E-STOP 해제로 리셋)')

    def _step(self):
        t = time.monotonic()
        dt = clamp(t - self.last_step, 1e-3, 0.2)
        self.last_step = t
        if self.pi is None:
            if t - self.last_reconnect > 2.0:
                self._connect()
            self._publish_status(t, force=False)
            return
        if not self.notify_alive:
            self._drop_connection('알림 스트림 중단')
            return

        sc = self.cfg['speed_control']
        self.warnings = []
        if not self.enc_verified:
            self.warnings.append('ENCODERS_UNVERIFIED: PI 속도제어·속도/odom·자동주차 차단 (Open-loop 수동 조종만 가능)')
        # --- 엔코더 스냅샷 & RPM ---
        with self.lock:
            snap = [(mo.enc.count, mo.enc.last_tick, mo.enc.errors) for mo in self.motors]
        win = sc['velocity_window_s']
        alpha = sc['rpm_filter_alpha']
        dist_step = []
        for mo, (cnt, ltick, _err) in zip(self.motors, snap):
            mo.samples.append((t, cnt, ltick))
            old = mo.samples[0]
            for smp in mo.samples:
                if t - smp[0] >= win:
                    old = smp
                else:
                    break
            while len(mo.samples) > 2 and t - mo.samples[1][0] >= win:
                mo.samples.popleft()
            dc = cnt - old[1]
            if dc == 0:
                raw = 0.0
            else:
                dt_us = tick_diff(ltick, old[2])
                span = dt_us * 1e-6 if 0 < dt_us < 5e6 else max(t - old[0], 1e-3)
                raw = dc / self.cpr / span * 60.0
            mo.measured_rpm = alpha * raw + (1 - alpha) * mo.measured_rpm
            dd = (cnt - mo.last_count) / self.cpr * math.pi * self.wheel_d
            mo.net_m += dd
            dist_step.append(dd)
            if cnt != mo.last_count:
                mo.last_move_t = t
            mo.last_count = cnt
        self.odometer += abs(sum(dist_step) / 2.0)

        # --- 명령 해석 ---
        cmd = self.cmd
        age = t - self.cmd_rx if cmd else None
        mode, reason = 'stop', None
        if cmd is None:
            reason = 'NO_COMMAND'
        elif age > self.cfg['safety']['hw_command_timeout_s']:
            reason = 'COMMAND_TIMEOUT'
        elif cmd.get('estop'):
            reason = 'ESTOP'
        else:
            mode = cmd.get('mode', 'stop')
        motors_ok = self.motors_disabled_reason is None and self.wd_ok
        if mode != 'stop' and not motors_ok:
            mode, reason = 'stop', 'MOTORS_DISABLED'
        if mode == 'velocity' and self.latched_faults:
            mode, reason = 'stop', 'FAULT_LATCHED'
        if mode == 'velocity' and not self.enc_verified:
            mode, reason = 'stop', 'ENCODERS_UNVERIFIED'

        # --- 조향 ---
        if cmd and reason not in ('NO_COMMAND', 'COMMAND_TIMEOUT'):
            lim = self.settings['max_steer_rad']
            self.steer_target = clamp(float(cmd.get('steer', 0.0)), -lim, lim)
        rate = self.st['rate_limit_rad_s'] * dt
        self.steer_cmd += clamp(self.steer_target - self.steer_cmd, -rate, rate)
        self.steer_cmd = clamp(self.steer_cmd, -self.steer_lim_right, self.steer_lim_left)
        sign = -1.0 if self.st.get('invert') else 1.0
        pulse = int(round(clamp(self.st['center_us'] + sign * self.steer_cmd * self.st['us_per_rad'],
                                *self.pulse_lim)))
        try:
            if pulse != self.steer_pulse:
                self.pi.set_servo(self.st['gpio'], pulse)
                self.steer_pulse = pulse
        except (OSError, PigpioError) as e:
            self._drop_connection(e)
            return

        # --- 구동 ---
        max_pwm = min(self.settings['max_pwm'], self.hard_max_pwm)
        slew = sc['pwm_slew_per_s'] * dt
        L = self.cfg['vehicle']['wheelbase_m']
        half_track = self.cfg['vehicle']['track_width_m'] / 2.0
        diff = [1.0, 1.0]
        if sc.get('electronic_differential', True) and L > 0:
            k = half_track * math.tan(self.steer_cmd) / L
            diff = [1.0 - k, 1.0 + k]           # 좌회전(+) → 오른쪽 바퀴가 빠름
        if mode == 'velocity':
            v = float(cmd.get('v', 0.0))
            rpm_c = v / (math.pi * self.wheel_d) * 60.0
            lim_rpm = self.settings['max_rpm']
            acc = sc['rpm_accel_limit'] * dt
            for mo, k in zip(self.motors, diff):
                tgt = clamp(rpm_c * k, -lim_rpm, lim_rpm)
                mo.target_rpm += clamp(tgt - mo.target_rpm, -acc, acc)
                e = mo.target_rpm - mo.measured_rpm
                if abs(mo.target_rpm) < 1e-6 and abs(mo.measured_rpm) < 2.0:
                    mo.integ, mo.duty = 0.0, 0.0          # 정지 목표: 적분기 리셋, 브레이크
                    continue
                ff = sc['kff'] * mo.target_rpm / self.cfg['motors']['no_load_rpm'] * self.ff_scale
                u_raw = ff + sc['kp'] * e + mo.integ
                u = clamp(u_raw, -max_pwm, max_pwm)
                u = mo.duty + clamp(u - mo.duty, -slew, slew)
                limited = abs(u - u_raw) > 1e-9
                # Anti-windup (조건부 적분): 포화/변화율 제한 중에는 포화를 풀어주는 방향으로만 적분
                if not limited or (e * (u_raw - u) < 0):
                    mo.integ = clamp(mo.integ + sc['ki'] * e * dt, -max_pwm, max_pwm)
                mo.duty = u
        elif mode == 'throttle':
            thr = clamp(float(cmd.get('throttle', 0.0)), -1.0, 1.0)
            for mo, k in zip(self.motors, diff):
                tgt = clamp(thr * max_pwm * k, -max_pwm, max_pwm)
                mo.duty += clamp(tgt - mo.duty, -slew, slew)
                mo.integ, mo.target_rpm = 0.0, 0.0
        else:
            for mo in self.motors:                   # 정지는 즉시 (변화율 제한 없음)
                mo.duty, mo.integ, mo.target_rpm = 0.0, 0.0, 0.0

        # --- 엔코더 이상 감지 ---
        for i, mo in enumerate(self.motors):
            if abs(mo.duty) >= sc['stall_pwm_threshold'] and t - mo.last_move_t > sc['stall_time_s']:
                code = f'ENCODER_OR_STALL_{mo.name}'
                if mode == 'velocity':
                    self._latch(code)
                else:
                    self.warnings.append(code)
            elif abs(mo.duty) < sc['stall_pwm_threshold']:
                mo.last_move_t = t          # 임계 미만 출력에서는 정지 판정 타이머를 리셋
            if abs(mo.duty) > 0.05 and mo.duty * mo.measured_rpm < 0 and \
                    abs(mo.measured_rpm) > sc['reverse_rpm_threshold']:
                mo.reverse_since = mo.reverse_since or t
                if t - mo.reverse_since > sc['reverse_time_s']:
                    code = f'DIRECTION_MISMATCH_{mo.name}'
                    if mode == 'velocity':
                        self._latch(code)
                    else:
                        self.warnings.append(code)
            else:
                mo.reverse_since = None
        if mode == 'velocity' and self.latched_faults:
            for mo in self.motors:
                mo.duty, mo.integ, mo.target_rpm = 0.0, 0.0, 0.0
            mode, reason = 'stop', 'FAULT_LATCHED'

        # --- 출력 ---
        try:
            for mo in self.motors:
                self._apply_duty(mo)
            if self.wd_sid is not None:
                self.pi.update_script(self.wd_sid, (1,))
        except (OSError, PigpioError) as e:
            self._drop_connection(e)
            return

        self.active_mode, self.stop_reason, self.cmd_age = mode, reason, age
        self._publish_fast(snap)
        self._publish_status(t, force=False)

    def _apply_duty(self, mo):
        rng = self.cfg['motors']['pwm_range']
        d = mo.duty if not mo.cfg.get('invert_output') else -mo.duty
        a = int(round(max(d, 0.0) * rng))
        b = int(round(max(-d, 0.0) * rng))
        if (a, b) == mo.applied:
            return
        # MDD3A 진리표(DS): A=PWM,B=L → 정방향 / A=L,B=PWM → 역방향 / L,L → 브레이크
        ga, gb = mo.cfg['pwm_a'], mo.cfg['pwm_b']
        if a == 0:
            self.pi.write(ga, 0)
        if b == 0:
            self.pi.write(gb, 0)
        if a:
            self.pi.set_pwm(ga, a)
        if b:
            self.pi.set_pwm(gb, b)
        mo.applied = (a, b)

    # ================================================================ 발행
    def _trailer_angle(self):
        if not self.trailer_cpr:
            return None
        return wrap_pi((self.trailer_enc.count - self.trailer_zero_count) * 2 * math.pi / self.trailer_cpr)

    def _publish_fast(self, snap):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        for mo, (cnt, _lt, _e) in zip(self.motors, snap):
            js.name.append(mo.name)
            js.position.append(cnt / self.cpr * 2 * math.pi)
            js.velocity.append(mo.measured_rpm * 2 * math.pi / 60.0)
            js.effort.append(mo.duty)       # effort 필드에 PWM duty(-1..1) 기록 (토크 아님)
        ta = self._trailer_angle()
        if ta is not None:
            js.name.append('trailer_joint')
            js.position.append(ta)
            js.velocity.append(0.0)
            js.effort.append(0.0)
            self.pub_trailer.publish(Float64(data=ta))
        self.pub_js.publish(js)
        self.pub_steer.publish(Float64(data=self.steer_cmd))

    def _publish_status(self, t, force):
        if t - self.last_status < 0.1 and not force:
            return
        self.last_status = t
        connected = self.pi is not None and self.notify_alive
        sc = self.cfg['speed_control']
        motors = []
        for mo in self.motors:
            motors.append({
                'name': mo.name,
                'target_rpm': round(mo.target_rpm, 1),
                'measured_rpm': round(mo.measured_rpm, 1) if connected else None,
                'pwm': round(mo.duty, 3),
                'count': mo.enc.count,
                'revolutions': round(mo.enc.count / self.cpr, 3),
                'direction': 'FWD' if mo.measured_rpm > 1 else 'REV' if mo.measured_rpm < -1 else 'STOP',
                'distance_m': round(mo.net_m, 3),
                'enc_errors': mo.enc.errors,
            })
        speeds = [mo.measured_rpm / 60.0 * math.pi * self.wheel_d for mo in self.motors]
        ta = self._trailer_angle()
        cal = []
        vc = self.cfg['vehicle']
        if not vc.get('wheelbase_measured'):
            cal.append('wheelbase_m')
        if not vc.get('track_width_measured'):
            cal.append('track_width_m')
        if not self.st.get('calibrated'):
            cal.append('steering')
        if not self.cfg['lidar'].get('measured'):
            cal.append('lidar_tf')
        if not self.trailer_cpr:
            cal.append('trailer.ppr')
        if not self.enc_verified:
            cal.append('motors.encoders_verified')
        if not self.us_cfg.get('mount_measured'):
            cal.append('ultrasonic_mount')
        faults = list(self.latched_faults)
        if not connected:
            faults.append('PIGPIO_DISCONNECTED')
        if self.motors_disabled_reason:
            faults.append('MOTORS_DISABLED')
        if not self.wd_ok:
            faults.append('WATCHDOG_SCRIPT_FAILED')
        status = {
            'stamp': time.time(),
            'pigpio_connected': connected,
            'edge_rate_hz': round(self.notify_rate),
            'motors_enabled': self.motors_disabled_reason is None and self.wd_ok and connected,
            'motors_disabled_reason': self.motors_disabled_reason,
            'hard_max_pwm': round(self.hard_max_pwm, 3),
            'rated_rpm': self.cfg['motors']['rated_rpm'],
            'counts_per_rev': self.cpr,
            'settings': {k: round(v, 4) for k, v in self.settings.items()},
            'steer_hw_limit_rad': round(self.steer_hw_limit, 4),
            'active_mode': self.active_mode,
            'stop_reason': self.stop_reason if connected else 'PIGPIO_DISCONNECTED',
            'cmd_age_s': finite_or_none(self.cmd_age, 3),
            'motors': motors,
            # 검증 전에는 속도를 측정값으로 내보내지 않음 (raw RPM/count 는 위 motors 에 그대로 표시)
            'speed_m_s': round(sum(speeds) / 2, 3) if connected and self.enc_verified else None,
            'encoders_verified': self.enc_verified,
            'odometer_m': round(self.odometer, 3),
            'net_distance_m': round(sum(mo.net_m for mo in self.motors) / 2, 3),
            # 엔코더 데이터 신뢰 가능 여부: 스트림 정상 + 엔코더 관련 고장 없음
            'encoders_ok': connected and self.enc_verified and
            not any(f.startswith(('ENCODER', 'DIRECTION')) for f in faults),
            'faults': faults,
            'warnings': self.warnings,
            'steering': {
                'target_rad': round(self.steer_target, 4),
                'cmd_rad': round(self.steer_cmd, 4),
                'pulse_us': self.steer_pulse,
                'measured_rad': None,          # 서보 위치 피드백 없음 — 측정값 아님을 명시
                'limit_left_rad': round(self.steer_lim_left, 4),
                'limit_right_rad': round(self.steer_lim_right, 4),
                'calibrated': bool(self.st.get('calibrated')),
            },
            'trailer': {
                'enabled': self.trailer_cpr is not None,
                'angle_rad': finite_or_none(ta),
                'angle_deg': finite_or_none(math.degrees(ta) if ta is not None else None, 2),
                'raw_count': self.trailer_enc.count,
                'zero_source': self.trailer_zero_source,
                'z_seen': self.z_count is not None,
                'z_period_counts': self.z_period,
                'enc_errors': self.trailer_enc.errors,
            },
            'ultrasonic': [{
                'name': u['cfg']['name'], 'label': u['cfg'].get('label', u['cfg']['name']),
                'range_m': finite_or_none(u['range'], 3), 'status': u['status'],
                'age_s': round(t - u['t'], 2) if u['t'] else None, 'fails': u['fails'],
            } for u in self.us],
            'calibration_pending': cal,
        }
        self.pub_status.publish(String(data=dumps(status)))

    # ================================================================ 초음파 (순차 측정 스레드)
    def _ultrasonic_loop(self):
        us = self.us_cfg
        c = us['speed_of_sound_m_s']
        while self.running:
            for i, u in enumerate(self.us):
                if not self.running:
                    return
                t0 = time.monotonic()
                s = u['cfg']
                rng, status = float('nan'), 'NO_PIGPIO'
                pi = self.pi
                if pi is not None and self.notify_alive:
                    with self.lock:
                        self.echo_width.pop(s['echo'], None)
                        self.echo_rise.pop(s['echo'], None)
                        self.echo_waiting = s['echo']
                        self.echo_event.clear()
                    try:
                        pi.trigger(s['trig'], 10, 1)
                        got = self.echo_event.wait(us['timeout_s'])
                    except (OSError, PigpioError):
                        got = False
                    with self.lock:
                        self.echo_waiting = None
                        w = self.echo_width.pop(s['echo'], None)
                    if got and w is not None:
                        d = w * 1e-6 * c / 2.0
                        if d < us['min_range_m']:
                            rng, status = float('-inf'), 'TOO_CLOSE'
                        elif d > us['max_range_m']:
                            rng, status = float('inf'), 'OUT_OF_RANGE'
                        else:
                            rng, status = d, 'OK'
                    else:
                        status = 'TIMEOUT'
                u['status'] = status
                u['range'] = rng if status == 'OK' else None
                if status == 'OK':
                    u['fails'] = 0
                else:
                    u['fails'] += 1
                u['t'] = time.monotonic()
                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f'ultrasonic_{s["name"]}'
                msg.radiation_type = Range.ULTRASOUND
                msg.field_of_view = float(us['field_of_view_rad'])
                msg.min_range = float(us['min_range_m'])
                msg.max_range = float(us['max_range_m'])
                msg.range = float(rng)   # REP-117: +inf 범위초과, -inf 근접, NaN 측정실패
                try:
                    self.pub_us[i].publish(msg)
                except Exception:
                    return
                rest = us['period_s'] - (time.monotonic() - t0)
                if rest > 0:
                    time.sleep(rest)

    # ================================================================ 종료
    def shutdown_outputs(self):
        self.running = False
        pi = self.pi
        if pi is None:
            return
        try:
            for mo in self.motors:
                pi.write(mo.cfg['pwm_a'], 0)
                pi.write(mo.cfg['pwm_b'], 0)
            pi.set_servo(self.st['gpio'], 0)
            if self.wd_sid is not None:
                pi.stop_script(self.wd_sid)
                pi.delete_script(self.wd_sid)
        except Exception:
            pass
        self._drop_connection('종료')


def main():
    rclpy.init()
    node = VehicleNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown_outputs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
