"""2D 라이다 스캔매칭 (Point-to-Line ICP) — 라이다만으로 차량 이동량(odom)을 추정하는 순수 계산 모듈.

ROS 의존성 없음(numpy 만 사용, scipy 가 있으면 최근접 탐색에 cKDTree 사용) → Windows 에서도 단위 테스트 가능.

좌표: 모든 점은 base_link(뒷차축 중심) 좌표로 변환한 뒤 정합한다 → 결과가 곧 base_link 의 이동량.
자세 표현: (x, y, th)  — 점 p 를 기준 프레임으로 옮기는 변환  p_ref = R(th)·p + [x, y]
"""
import math

import numpy as np

try:  # 선택 사항: 있으면 빠름, 없으면 numpy 전수 비교
    from scipy.spatial import cKDTree
except Exception:  # pragma: no cover
    cKDTree = None


# ===================================================================== 스캔 → 점
def beam_mask(n, angle_min, angle_inc, ranges, rmin, rmax, sectors_rad=()):
    """사용할 빔 True. sectors_rad: [(시작, 끝), ...] 라이다 프레임 각도(rad, 반시계) 구간은 제외."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(n)
    ok = np.isfinite(r) & (r >= rmin) & (r <= rmax)
    for lo, hi in sectors_rad:
        span = (hi - lo) % (2 * math.pi)
        ok &= ((a - lo) % (2 * math.pi)) > span
    return ok, r, a


def to_base_points(r, a, mount):
    """라이다 극좌표 → base_link 직교좌표. mount = (x, y, yaw) 라이다 장착 위치."""
    mx, my, myaw = mount
    lx, ly = r * np.cos(a), r * np.sin(a)
    c, s = math.cos(myaw), math.sin(myaw)
    return np.column_stack((mx + c * lx - s * ly, my + s * lx + c * ly))


def voxel_downsample(pts, voxel):
    """격자 칸마다 첫 점만 유지 (스캔 각도 순서 보존 → 이웃 점으로 법선 계산 가능)."""
    if voxel <= 0 or len(pts) == 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


def limit_points(pts, max_points):
    if max_points and len(pts) > max_points:
        idx = np.linspace(0, len(pts) - 1, max_points).round().astype(int)
        return pts[idx]
    return pts


def line_normals(pts, k=2, max_neighbor_m=0.25, max_flatness=0.15):
    """각 점의 국소 직선 법선 (앞뒤 k 개 이웃 PCA). 벽/모서리처럼 직선이 아닌 곳은 valid=False."""
    n = len(pts)
    normals = np.zeros((n, 2))
    valid = np.zeros(n, dtype=bool)
    if n < 2 * k + 1:
        return normals, valid
    offs = np.arange(-k, k + 1)
    idx = np.clip(np.arange(n)[:, None] + offs[None, :], 0, n - 1)
    nb = pts[idx]                                         # (n, 2k+1, 2)
    near = np.linalg.norm(nb - pts[:, None, :], axis=2).max(axis=1) <= max_neighbor_m
    mu = nb.mean(axis=1)
    d = nb - mu[:, None, :]
    sxx, syy, sxy = (d[..., 0] ** 2).mean(1), (d[..., 1] ** 2).mean(1), (d[..., 0] * d[..., 1]).mean(1)
    tr, det = sxx + syy, sxx * syy - sxy ** 2
    disc = np.sqrt(np.maximum(tr * tr / 4 - det, 0))
    l_small, l_big = tr / 2 - disc, tr / 2 + disc
    # 작은 고유값의 고유벡터 = 법선
    nx, ny = sxy, l_small - sxx
    alt = np.abs(nx) + np.abs(ny) < 1e-12
    nx = np.where(alt, l_small - syy, nx)
    ny = np.where(alt, sxy, ny)
    norm = np.hypot(nx, ny)
    ok = near & (norm > 1e-12) & (l_big > 1e-8) & (l_small <= max_flatness ** 2 * l_big)
    normals[ok, 0] = nx[ok] / norm[ok]
    normals[ok, 1] = ny[ok] / norm[ok]
    valid[ok] = True
    return normals, valid


# ===================================================================== 자세 연산
def compose(a, b):
    """a ∘ b  (b 를 먼저 적용한 뒤 a)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2]))


def wrap(t):
    return math.atan2(math.sin(t), math.cos(t))


def transform(pts, T):
    c, s = math.cos(T[2]), math.sin(T[2])
    return np.column_stack((c * pts[:, 0] - s * pts[:, 1] + T[0], s * pts[:, 0] + c * pts[:, 1] + T[1]))


# ===================================================================== 기준(키프레임) 스캔
class Reference:
    def __init__(self, pts):
        self.pts = pts
        self.normals, self.valid = line_normals(pts)
        self.tree = cKDTree(pts) if (cKDTree is not None and len(pts)) else None

    def nearest(self, q):
        if self.tree is not None:
            d, j = self.tree.query(q, k=1)
            return d, j
        d2 = ((q[:, None, :] - self.pts[None, :, :]) ** 2).sum(axis=2)
        j = d2.argmin(axis=1)
        return np.sqrt(d2[np.arange(len(q)), j]), j


# ===================================================================== ICP
def icp(ref, src, init, p):
    """src(현재 스캔, base_link) 를 ref(키프레임) 에 정합. init: 초기 추정 (x, y, th).
    반환 dict: ok, pose, matches, inlier_ratio, rms, degenerate, min_eig, iters, reason"""
    out = {'ok': False, 'pose': tuple(init), 'matches': 0, 'inlier_ratio': 0.0, 'rms': None,
           'degenerate': False, 'min_eig': None, 'iters': 0, 'reason': ''}
    if len(ref.pts) < p['min_matches'] or len(src) < p['min_matches']:
        out['reason'] = 'TOO_FEW_POINTS'
        return out
    T = [float(init[0]), float(init[1]), float(init[2])]
    huber = p['huber_m']
    corr = p['max_corr_m']
    use = np.zeros(len(src), dtype=bool)
    for it in range(p['max_iterations']):
        out['iters'] = it + 1
        q = transform(src, T)
        d, j = ref.nearest(q)
        use = (d < corr) & ref.valid[j]
        if use.sum() < p['min_matches']:
            out['reason'] = 'TOO_FEW_MATCHES'
            out['matches'] = int(use.sum())
            return out
        n = ref.normals[j[use]]
        qq = q[use]
        r = np.einsum('ij,ij->i', n, qq - ref.pts[j[use]])
        w = np.where(np.abs(r) <= huber, 1.0, huber / np.maximum(np.abs(r), 1e-12))
        J = np.column_stack((n[:, 0], n[:, 1], n[:, 0] * -qq[:, 1] + n[:, 1] * qq[:, 0]))
        H = (J * w[:, None]).T @ J
        g = (J * w[:, None]).T @ r
        H[2, 2] += 1e-6
        try:
            dx, dy, dth = -np.linalg.solve(H, g)
        except np.linalg.LinAlgError:
            out['reason'] = 'SINGULAR'
            return out
        c, s = math.cos(dth), math.sin(dth)
        T = [c * T[0] - s * T[1] + dx, s * T[0] + c * T[1] + dy, wrap(T[2] + dth)]
        corr = max(p['min_corr_m'], corr * p['corr_decay'])
        if math.hypot(dx, dy) < 1e-4 and abs(dth) < 1e-4:
            break
    # 최종 품질 평가 (수렴 자세 기준)
    q = transform(src, T)
    d, j = ref.nearest(q)
    use = (d < p['min_corr_m'] * 2) & ref.valid[j]
    m = int(use.sum())
    out['matches'] = m
    out['inlier_ratio'] = m / float(len(src))
    out['pose'] = (T[0], T[1], T[2])
    if m < p['min_matches']:
        out['reason'] = 'TOO_FEW_MATCHES'
        return out
    n = ref.normals[j[use]]
    r = np.einsum('ij,ij->i', n, q[use] - ref.pts[j[use]])
    out['rms'] = float(np.sqrt((r ** 2).mean()))
    # 퇴화 판정: 법선 방향 분포의 최소 고유값 (복도처럼 한 방향 벽만 보이면 그 방향 이동을 못 잼)
    ev = np.linalg.eigvalsh((n.T @ n) / m)
    out['min_eig'] = float(ev[0])
    out['degenerate'] = bool(ev[0] < p['degenerate_min_eig'])
    if out['inlier_ratio'] < p['min_inlier_ratio']:
        out['reason'] = 'LOW_OVERLAP'
    elif out['rms'] > p['max_rms_m']:
        out['reason'] = 'HIGH_RESIDUAL'
    else:
        out['ok'] = True
    return out


DEFAULTS = {
    'voxel_m': 0.04, 'max_points': 400, 'max_iterations': 25,
    'max_corr_m': 0.30, 'min_corr_m': 0.05, 'corr_decay': 0.7, 'huber_m': 0.03,
    'min_matches': 40, 'min_inlier_ratio': 0.35, 'max_rms_m': 0.03, 'degenerate_min_eig': 0.03,
    'keyframe_dist_m': 0.15, 'keyframe_angle_rad': 0.17, 'keyframe_min_ratio': 0.6,
    'max_step_m': 0.30, 'max_step_rad': 0.50, 'lost_reset_scans': 5, 'speed_filter_s': 0.3,
}


class LaserOdometry:
    """스캔-키프레임 정합으로 누적 자세를 추정. 실패한 스캔은 자세를 갱신하지 않는다(추정값으로 메우지 않음)."""

    def __init__(self, params=None):
        self.p = dict(DEFAULTS)
        self.p.update({k: v for k, v in (params or {}).items() if v is not None})
        self.pose = (0.0, 0.0, 0.0)          # odom 프레임 기준 base_link
        self.kf = None                       # Reference
        self.kf_pose = None                  # 키프레임 시점 odom 자세
        self.rel = (0.0, 0.0, 0.0)           # 키프레임 기준 최근 자세
        self.step_guess = (0.0, 0.0, 0.0)    # 직전 스캔 간 이동 (초기 추정용)
        self.t = None
        self.v = 0.0                         # 전진 속도 (base_link x, 후진은 음수)
        self.w = 0.0
        self.fails = 0
        self.resets = 0
        self.last = {}

    def prepare(self, pts):
        return limit_points(voxel_downsample(pts, self.p['voxel_m']), self.p['max_points'])

    def update(self, pts, t):
        """pts: base_link 점 (N,2), t: 스캔 시각[s]. 반환 dict(status, ok, ...)."""
        p = self.p
        pts = self.prepare(pts)
        if self.kf is None:
            if len(pts) < p['min_matches']:
                self.last = {'status': 'NO_DATA', 'ok': False, 'points': len(pts)}
                return self.last
            self._set_keyframe(pts)
            self.t = t
            self.last = {'status': 'INIT', 'ok': True, 'points': len(pts)}
            return self.last
        dt = t - self.t if self.t is not None else 0.0
        init = compose(self.rel, self.step_guess)
        res = icp(self.kf, pts, init, p)
        step = compose(inverse(self.rel), res['pose'])
        if res['ok'] and (math.hypot(step[0], step[1]) > p['max_step_m'] or abs(step[2]) > p['max_step_rad']):
            res['ok'], res['reason'] = False, 'JUMP'
        info = {'points': len(pts), 'matches': res['matches'], 'inlier_ratio': round(res['inlier_ratio'], 3),
                'rms_m': None if res['rms'] is None else round(res['rms'], 4),
                'degenerate': res['degenerate'], 'iters': res['iters']}
        if not res['ok']:
            self.fails += 1
            info.update(status='LOST', ok=False, reason=res['reason'], fails=self.fails)
            if self.fails >= p['lost_reset_scans']:
                # 연속 실패: 현재 스캔으로 키프레임을 다시 잡는다. 놓친 구간의 이동은 알 수 없으므로
                # odom 자세는 마지막 정상값에서 이어지며(점프 가능), 이 사실을 resets 로 알린다.
                self._set_keyframe(pts)
                self.resets += 1
                self.step_guess = (0.0, 0.0, 0.0)
                self.v = self.w = 0.0
                info['status'] = 'RESET'
                self.t = t
            self.last = info
            return info
        self.fails = 0
        self.rel = res['pose']
        self.pose = compose(self.kf_pose, self.rel)
        if dt > 1e-3:
            a = min(1.0, dt / max(p['speed_filter_s'], 1e-3))
            self.v += a * (step[0] / dt - self.v)
            self.w += a * (step[2] / dt - self.w)
            self.step_guess = step if dt < 0.5 else (0.0, 0.0, 0.0)
        self.t = t
        moved = math.hypot(self.rel[0], self.rel[1]) > p['keyframe_dist_m'] or abs(self.rel[2]) > p['keyframe_angle_rad']
        if moved or res['inlier_ratio'] < p['keyframe_min_ratio']:
            self._set_keyframe(pts)
        info.update(status='DEGENERATE' if res['degenerate'] else 'OK', ok=True)
        self.last = info
        return info

    def _set_keyframe(self, pts):
        self.kf = Reference(pts)
        self.kf_pose = self.pose
        self.rel = (0.0, 0.0, 0.0)
