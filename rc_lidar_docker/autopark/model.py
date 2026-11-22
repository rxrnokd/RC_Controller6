"""RC카 + 트레일러 운동학 모델, 차체 충돌 영역, 점유 격자 지도.

상태 x = [x, y, th, beta]
  (x, y)  견인차 뒷차축 중심 (Pi base_link 원점과 동일), th = 견인차 방향
  beta    = th_trailer - th  (MT6701 /trailer/angle 과 같은 부호: 트레일러가 차에 대해 반시계 +)
입력  v [m/s] (부호 = 전/후진), delta [rad] (+ = 좌회전)
견인차: 자전거 모델.  트레일러: 오프액슬 히치(뒷차축 뒤 M) + 1축 트레일러(히치→차축 L2)
  th_dot  = v tan(delta) / L
  th2_dot = (v sin(th - th2) - M th_dot cos(th - th2)) / L2
"""
import math
from dataclasses import dataclass

import numpy as np

REQUIRED_KEYS = [
    ('vehicle', 'wheelbase_m'), ('vehicle', 'steer_max_rad'), ('vehicle', 'length_m'),
    ('vehicle', 'width_m'), ('vehicle', 'rear_overhang_m'),
    ('trailer', 'hitch_offset_m'), ('trailer', 'length_m'), ('trailer', 'front_m'),
    ('trailer', 'rear_m'), ('trailer', 'width_m'), ('trailer', 'hitch_angle_max_rad'),
]


@dataclass
class Geometry:
    wheelbase: float
    steer_max: float
    car_length: float
    car_width: float
    car_rear: float
    hitch_offset: float
    trailer_length: float
    trailer_front: float
    trailer_rear: float
    trailer_width: float
    hitch_max: float

    def to_dict(self):
        return dict(self.__dict__)


def geometry_from_config(cfg):
    """(Geometry 또는 None, 누락/잘못된 항목 리스트)."""
    missing = []
    for sec, key in REQUIRED_KEYS:
        v = (cfg.get(sec) or {}).get(key)
        if v is None or not isinstance(v, (int, float)) or not math.isfinite(v):
            missing.append(f'{sec}.{key}')
        elif key != 'hitch_offset_m' and v <= 0:
            missing.append(f'{sec}.{key} (> 0 이어야 함)')
    if missing:
        return None, missing
    v, t = cfg['vehicle'], cfg['trailer']
    return Geometry(v['wheelbase_m'], v['steer_max_rad'], v['length_m'], v['width_m'], v['rear_overhang_m'],
                    t['hitch_offset_m'], t['length_m'], t['front_m'], t['rear_m'], t['width_m'],
                    t['hitch_angle_max_rad']), []


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ------------------------------------------------------------------ 운동학
def deriv(s, v, delta, g):
    x, y, th, b = s
    thd = v * math.tan(delta) / g.wheelbase
    th2d = (-v * math.sin(b) - g.hitch_offset * thd * math.cos(b)) / g.trailer_length
    return (v * math.cos(th), v * math.sin(th), thd, th2d - thd)


def step(s, v, delta, dt, g):
    """RK4 한 스텝. s: 길이 4 시퀀스 → tuple."""
    k1 = deriv(s, v, delta, g)
    s2 = [s[i] + 0.5 * dt * k1[i] for i in range(4)]
    k2 = deriv(s2, v, delta, g)
    s3 = [s[i] + 0.5 * dt * k2[i] for i in range(4)]
    k3 = deriv(s3, v, delta, g)
    s4 = [s[i] + dt * k3[i] for i in range(4)]
    k4 = deriv(s4, v, delta, g)
    out = [s[i] + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(4)]
    out[2] = wrap(out[2])
    out[3] = wrap(out[3])
    return tuple(out)


def roll_arc(s, direction, delta, length, ds, g):
    """호 길이 파라미터 적분(속도 1 m/s 와 동치). 각 substep 끝 상태 리스트."""
    n = max(1, int(round(length / ds)))
    h = length / n
    out = []
    for _ in range(n):
        s = step(s, direction, delta, h, g)
        out.append(s)
    return out


def trailer_pose(s, g):
    x, y, th, b = s
    hx, hy = x - g.hitch_offset * math.cos(th), y - g.hitch_offset * math.sin(th)
    th2 = th + b
    return hx - g.trailer_length * math.cos(th2), hy - g.trailer_length * math.sin(th2), wrap(th2)


def state_from_trailer_goal(xt, yt, th2, g):
    """목표 = 트레일러 차축 자세, 연결각 0 (일직선 주차) → 전체 상태."""
    c, s = math.cos(th2), math.sin(th2)
    hx, hy = xt + g.trailer_length * c, yt + g.trailer_length * s
    return (hx + g.hitch_offset * c, hy + g.hitch_offset * s, wrap(th2), 0.0)


def _rect_circles(length, width):
    """사각형(길이 방향 x)을 덮는 원: (중심 x 오프셋 리스트, 반경)."""
    n = max(1, math.ceil(length / (0.5 * width)))
    seg = length / n
    r = math.hypot(width / 2.0, seg / 2.0)
    return [(-length / 2.0 + seg * (i + 0.5)) for i in range(n)], r


class Footprint:
    """견인차·트레일러 사각형을 원들로 근사 (보수적, 항상 사각형을 포함)."""

    def __init__(self, g):
        self.g = g
        offs, self.r_car = _rect_circles(g.car_length, g.car_width)
        c_mid = g.car_length / 2.0 - g.car_rear            # 뒷차축 기준 차체 중심
        self.car_offs = [c_mid + o for o in offs]
        tl = g.trailer_front + g.trailer_rear
        offs, self.r_tr = _rect_circles(tl, g.trailer_width)
        t_mid = (g.trailer_front - g.trailer_rear) / 2.0   # 트레일러 차축 기준 중심
        self.tr_offs = [t_mid + o for o in offs]

    def circles(self, s):
        x, y, th, _b = s
        c, sn = math.cos(th), math.sin(th)
        xs = [x + o * c for o in self.car_offs]
        ys = [y + o * sn for o in self.car_offs]
        rs = [self.r_car] * len(self.car_offs)
        tx, ty, th2 = trailer_pose(s, self.g)
        c2, s2 = math.cos(th2), math.sin(th2)
        xs += [tx + o * c2 for o in self.tr_offs]
        ys += [ty + o * s2 for o in self.tr_offs]
        rs += [self.r_tr] * len(self.tr_offs)
        return xs, ys, rs

    def outline(self, s):
        """표시용 사각형 꼭짓점 [(car 4점), (trailer 4점)]."""
        g = self.g
        x, y, th, _ = s

        def rect(cx, cy, a, front, rear, w):
            c, sn = math.cos(a), math.sin(a)
            pts = [(front, w / 2), (front, -w / 2), (-rear, -w / 2), (-rear, w / 2)]
            return [(round(cx + px * c - py * sn, 3), round(cy + px * sn + py * c, 3)) for px, py in pts]
        tx, ty, th2 = trailer_pose(s, g)
        return [rect(x, y, th, g.car_length - g.car_rear, g.car_rear, g.car_width),
                rect(tx, ty, th2, g.trailer_front, g.trailer_rear, g.trailer_width)]


# ------------------------------------------------------------------ 지도
class GridMap:
    """점유 격자 (ROS 규약: row 0 = origin y, col 0 = origin x).  dist = 가장 가까운 장애물까지 거리[m]."""

    def __init__(self, occupied, resolution, origin_x, origin_y):
        from scipy.ndimage import distance_transform_edt
        self.occ = np.asarray(occupied, dtype=bool)
        self.h, self.w = self.occ.shape
        self.res = float(resolution)
        self.ox, self.oy = float(origin_x), float(origin_y)
        if self.occ.any():
            # 셀 중심 간 거리 - 반 셀 (장애물 셀의 실제 경계까지 보수적으로)
            self.dist = np.maximum(distance_transform_edt(~self.occ) * self.res - self.res / 2, 0.0)
        else:
            self.dist = np.full(self.occ.shape, 1e3)

    @classmethod
    def from_values(cls, values, width, height, resolution, ox, oy, occ_thresh=65, unknown_is_obstacle=True):
        a = np.asarray(values, dtype=np.int16).reshape(height, width)
        occ = a >= occ_thresh
        if unknown_is_obstacle:
            occ |= a < 0
        return cls(occ, resolution, ox, oy)

    @classmethod
    def from_ws_map(cls, m, cfg_map):
        import base64
        import zlib
        raw = np.frombuffer(zlib.decompress(base64.b64decode(m['data'])), dtype=np.int8)
        return cls.from_values(raw, m['width'], m['height'], m['resolution'], m['origin']['x'], m['origin']['y'],
                               cfg_map.get('occupied_threshold', 65), cfg_map.get('unknown_is_obstacle', True))

    @classmethod
    def from_pgm_yaml(cls, yaml_path, cfg_map):
        """map_server 형식 (Pi 웹 UI '지도 저장' 결과물) 로드."""
        import os
        import yaml
        with open(yaml_path, 'r', encoding='utf-8') as f:
            meta = yaml.safe_load(f)
        img_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
        with open(img_path, 'rb') as f:
            data = f.read()
        tokens, idx = [], 0
        while len(tokens) < 4:                     # P5 헤더 (주석 허용)
            while data[idx:idx + 1].isspace():
                idx += 1
            if data[idx:idx + 1] == b'#':
                while data[idx:idx + 1] not in (b'\n', b''):
                    idx += 1
                continue
            j = idx
            while not data[j:j + 1].isspace():
                j += 1
            tokens.append(data[idx:j])
            idx = j
        if tokens[0] != b'P5':
            raise ValueError('P5 PGM 만 지원')
        w, h, maxv = int(tokens[1]), int(tokens[2]), int(tokens[3])
        px = np.frombuffer(data[idx + 1: idx + 1 + w * h], dtype=np.uint8).reshape(h, w)[::-1].astype(float)
        p = px / maxv if meta.get('negate', 0) else (maxv - px) / maxv
        occ = p > meta.get('occupied_thresh', 0.65)
        if cfg_map.get('unknown_is_obstacle', True):
            occ |= (p >= meta.get('free_thresh', 0.196)) & ~occ
        o = meta['origin']
        return cls(occ, meta['resolution'], o[0], o[1])

    @classmethod
    def empty(cls, width_m, height_m, res=0.05, ox=0.0, oy=0.0, border=True):
        w, h = int(round(width_m / res)), int(round(height_m / res))
        occ = np.zeros((h, w), dtype=bool)
        if border:
            occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
        return cls(occ, res, ox, oy)

    def dist_at(self, x, y):
        c = int((x - self.ox) / self.res)
        r = int((y - self.oy) / self.res)
        if 0 <= r < self.h and 0 <= c < self.w:
            return self.dist[r, c]
        return 0.0                                  # 지도 밖 = 장애물

    def dist_interp(self, x, y):
        """쌍선형 보간 거리 (최적화 기울기용 — 연속 함수)."""
        fx, fy = (x - self.ox) / self.res - 0.5, (y - self.oy) / self.res - 0.5
        c0, r0 = math.floor(fx), math.floor(fy)
        if not (0 <= r0 < self.h - 1 and 0 <= c0 < self.w - 1):
            return 0.0
        ax, ay = fx - c0, fy - r0
        d = self.dist
        return ((1 - ax) * (1 - ay) * d[r0, c0] + ax * (1 - ay) * d[r0, c0 + 1]
                + (1 - ax) * ay * d[r0 + 1, c0] + ax * ay * d[r0 + 1, c0 + 1])

    def circle_clearances(self, s, fp):
        xs, ys, rs = fp.circles(s)
        return [self.dist_interp(x, y) - r for x, y, r in zip(xs, ys, rs)]

    def clearance(self, s, fp):
        xs, ys, rs = fp.circles(s)
        return min(self.dist_at(x, y) - r for x, y, r in zip(xs, ys, rs))

    def collides(self, s, fp, margin):
        xs, ys, rs = fp.circles(s)
        for x, y, r in zip(xs, ys, rs):
            if self.dist_at(x, y) < r + margin:
                return True
        return False
