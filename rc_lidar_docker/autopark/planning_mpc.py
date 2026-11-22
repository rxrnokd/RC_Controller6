"""② Planning MPC — Hybrid A* 경로를 구간(전진/후진)별로 최적화하고 Tracking MPC 용 기준 궤적·속도 프로파일 생성.

각 구간: 결정변수 = 조향각 시퀀스 delta_0..N-1 (호 길이 간격 ds 고정, 방향 고정)
  최소화  w_ref·Σ|x_k - x_ref,k|² + w_terminal·|x_N - 목표|² + w_steer·Σδ² + w_steer_rate·ΣΔδ²
          + w_obstacle·Σ max(0, soft - 여유거리)² + w_hitch·Σ max(0, |β| - β_soft)²
  제약    |δ| ≤ steer_max,  |δ_{k+1} - δ_k| ≤ steer_rate_max · ds / v_max(방향)
최적화 결과가 충돌/연결각 한계/목표 오차 검사를 통과하지 못하면 Hybrid A* 경로를 그대로 기준 궤적으로 사용
(mpc_ok=False 로 표시). 속도 프로파일: 방향별 최대속도, 가속도, 조향 변화율 제한, 구간 끝(전환점) 정지.
"""
import math

import numpy as np
from scipy.optimize import minimize

from .model import Footprint, step as fstep, trailer_pose, wrap


class RefTrajectory:
    """Tracking MPC 기준 궤적. 각 구간은 독립 인덱스 범위 [i0, i1] (전환점은 양 구간에 중복)."""

    def __init__(self):
        self.states = np.zeros((0, 4))
        self.delta = np.zeros(0)
        self.v = np.zeros(0)          # 부호 있는 속도
        self.t = np.zeros(0)
        self.segments = []            # [{'i0','i1','dir'}]
        self.mpc_ok = False
        self.notes = []
        self.length = 0.0
        self.goal = None

    def summary(self):
        return {'points': len(self.v), 'segments': len(self.segments), 'mpc_ok': self.mpc_ok,
                'length_m': round(float(self.length), 3), 'duration_s': round(float(self.t[-1]), 2) if len(self.t) else 0,
                'directions': [s['dir'] for s in self.segments], 'notes': self.notes}


class PlanningMPC:
    def __init__(self, geom, grid, cfg):
        self.g = geom
        self.grid = grid
        self.c = cfg['planning_mpc']
        self.sp = cfg['speed']
        self.ha = cfg['hybrid_astar']
        self.margin = cfg['map']['safety_margin_m']
        self.fp = Footprint(geom)

    # ------------------------------------------------------------------ 오차 벡터
    def _err(self, s, r):
        a = self.c['angle_scale_m']
        return (s[0] - r[0], s[1] - r[1], a * wrap(s[2] - r[2]), a * wrap(s[3] - r[3]))

    def _sim(self, x0, d, u, ds):
        out = [tuple(x0)]
        s = tuple(x0)
        for uj in u:
            s = fstep(s, d, uj, ds, self.g)
            out.append(s)
        return out

    @staticmethod
    def _resample(states, steer, n):
        """A* 구간 점들을 n 등분으로 리샘플 (각도는 unwrap 후 보간)."""
        arr = np.array(states, dtype=float)
        arr[:, 2] = np.unwrap(arr[:, 2])
        arr[:, 3] = np.unwrap(arr[:, 3])
        src = np.linspace(0.0, 1.0, len(arr))
        dst = np.linspace(0.0, 1.0, n + 1)
        out = np.column_stack([np.interp(dst, src, arr[:, k]) for k in range(4)])
        st = np.array(steer, dtype=float)
        u = np.array([st[min(len(st) - 1, int(round(f * (len(st) - 1))))] for f in dst[1:]])
        return out, u

    def _optimize_segment(self, x0, ref, u_init, d, ds, target, is_last):
        """결정변수 z = [δ_0..δ_{n-1}, λ]  (λ = 구간 길이 배율, ds_실제 = ds·λ).  반환 (δ, 상태열, ds_실제)."""
        g, c = self.g, self.c
        n = len(u_init)
        lam_lo, lam_hi = c['length_scale_min'], c['length_scale_max']
        vmax = self.sp['v_max_forward_m_s'] if d > 0 else self.sp['v_max_reverse_m_s']
        rate = self.sp['steer_rate_max_rad_s'] * ds * lam_lo / vmax      # 가장 짧은 ds 기준 (보수적)
        hsoft = c['hitch_soft_ratio'] * g.hitch_max

        def sim(z):
            return self._sim(x0, d, z[:n], ds * z[n])

        def cost(z):
            u = z[:n]
            sts = sim(z)
            j = 0.0
            for k in range(1, n + 1):
                e = self._err(sts[k], ref[k])
                j += c['w_ref'] * (e[0] ** 2 + e[1] ** 2 + e[2] ** 2 + e[3] ** 2)
                for cl in self.grid.circle_clearances(sts[k], self.fp):   # 원별 연속 페널티
                    cl -= self.margin
                    if cl < c['obstacle_soft_m']:
                        j += c['w_obstacle'] * (c['obstacle_soft_m'] - cl) ** 2
                hb = abs(sts[k][3]) - hsoft
                if hb > 0:
                    j += c['w_hitch'] * hb ** 2
            e = self._err(sts[-1], target)
            j += c['w_terminal'] * (e[0] ** 2 + e[1] ** 2 + e[2] ** 2 + e[3] ** 2)
            j += c['w_steer'] * float(np.dot(u, u)) * ds
            du = np.diff(u)
            j += c['w_steer_rate'] * float(np.dot(du, du))
            return j

        A = np.zeros((max(0, 2 * (n - 1)), n + 1))
        for k in range(n - 1):
            A[2 * k, k], A[2 * k, k + 1] = 1.0, -1.0       # rate - (u_{k+1}-u_k) >= 0
            A[2 * k + 1, k], A[2 * k + 1, k + 1] = -1.0, 1.0
        cons = [{'type': 'ineq', 'fun': lambda z: rate + A @ z, 'jac': lambda z: A}] if n > 1 else []
        # 구간 끝 자세 = 목표(전환점/주차 목표) — 등식 제약 (실패 시 연결부 검사에서 걸러짐)
        cons.append({'type': 'eq', 'fun': lambda z: np.array(self._err(sim(z)[-1], target))})
        z0 = np.append(np.clip(u_init, -g.steer_max, g.steer_max), 1.0)
        res = minimize(cost, z0, method='SLSQP', bounds=[(-g.steer_max, g.steer_max)] * n + [(lam_lo, lam_hi)],
                       constraints=cons, options={'maxiter': c['max_iter'], 'ftol': 1e-7, 'eps': 1e-5})
        z = res.x.copy()
        z[:n] = np.clip(z[:n], -g.steer_max, g.steer_max)
        z[n] = min(max(z[n], lam_lo), lam_hi)
        return z[:n], sim(z), ds * z[n]

    def _check(self, sts):
        for s in sts:
            if abs(s[3]) > self.g.hitch_max:
                return 'hitch_limit'
            if self.grid.collides(s, self.fp, self.margin):
                return 'collision'
        return None

    # ------------------------------------------------------------------
    def _gap(self, a, b):
        return (math.hypot(a[0] - b[0], a[1] - b[1]), abs(wrap(a[2] - b[2])), abs(wrap(a[3] - b[3])))

    def _gap_ok(self, gap, tol_prefix):
        c = self.c
        return gap[0] <= c[tol_prefix + 'xy_m'] and gap[1] <= c[tol_prefix + 'yaw_rad'] and             gap[2] <= c[tol_prefix + 'hitch_rad']

    def optimize(self, astar, goal):
        """astar: hybrid_astar.PlanResult (ok=True), goal: 목표 전체 상태.

        전진 구간은 시작점을 고정하고 앞으로 적분, 후진 구간은 끝점을 고정하고 시간 역방향(= 전진, 안정)으로 적분.
        (트레일러 후진은 개루프 불안정이라 단일 슈팅 최적화가 발산하기 때문.) 마지막 후진 구간의 끝 = 목표(정확).
        구간 연결부(전환점·시작점) 불일치는 join_tol 이내여야 하며, 아니면 Hybrid A* 경로로 대체."""
        segs = astar.segments()
        sub = self.ha['step_m'] / max(1, int(round(self.ha['step_m'] / self.ha['substep_m'])))
        pieces, notes, bad = [], [], None
        for k, (i0, i1, d) in enumerate(segs):
            length = (i1 - i0) * sub
            n = int(min(self.c['max_vars_per_segment'], max(3, round(length / self.c['ds_m']))))
            ds = length / n
            ref, u_init = self._resample(astar.states[i0:i1 + 1], astar.steer[i0 + 1:i1 + 1], n)
            a_start = tuple(astar.states[0]) if k == 0 else tuple(astar.states[i0])
            a_end = tuple(goal) if k == len(segs) - 1 else tuple(astar.states[i1])
            if d > 0:
                u, sts, ds = self._optimize_segment(a_start, ref, u_init, 1, ds, a_end, False)
            else:
                u_r, sts_r, ds = self._optimize_segment(a_end, ref[::-1], u_init[::-1], 1, ds, a_start, False)
                u, sts = u_r[::-1], sts_r[::-1]
            why = self._check(sts)
            if why:
                bad = f'구간 {k}: {why}'
                break
            pieces.append((np.array(sts), np.append(u, u[-1]), d, ds))
        if bad is None:
            joins = [('시작', astar.states[0], pieces[0][0][0], 'join_tol_')]
            joins += [(f'전환점 {k}', pieces[k][0][-1], pieces[k + 1][0][0], 'join_tol_') for k in range(len(pieces) - 1)]
            joins += [('목표', pieces[-1][0][-1], goal, 'final_tol_')]
            for name, a, b, tol in joins:
                gp = self._gap(a, b)
                if not self._gap_ok(gp, tol):
                    bad = f'{name} 불일치 {gp[0] * 100:.1f} cm / {math.degrees(gp[1]):.1f}° / 연결각 {math.degrees(gp[2]):.1f}°'
                    break
        mpc_ok = bad is None
        if not mpc_ok:                    # 하나라도 실패하면 전체를 A* 경로로 (구간 간 연속성 보장)
            notes.append(f'최적화 결과 거부: {bad} → Hybrid A* 경로 사용')
            pieces = []
            for i0, i1, d in segs:
                sts = np.array(astar.states[i0:i1 + 1], dtype=float)
                u = np.array(astar.steer[i0 + 1:i1 + 1] + [astar.steer[i1]], dtype=float)
                pieces.append((sts, u, d, sub))
        traj = self._build(pieces)
        traj.mpc_ok = mpc_ok
        traj.goal = tuple(goal)
        if not mpc_ok:   # A* 경로는 목표에 허용오차 내로만 도달 → 추종 도착 판정은 A* 끝 자세 기준
            traj.goal = tuple(astar.states[-1])
            tx, ty, th2 = trailer_pose(traj.goal, self.g)
            gx, gy, gth = trailer_pose(goal, self.g)
            notes.append(f'A* 종료 자세와 지정 목표 차이: {math.hypot(tx - gx, ty - gy) * 100:.1f} cm, '
                         f'{math.degrees(abs(wrap(th2 - gth))):.1f}°')
        traj.notes = notes
        return traj

    # ------------------------------------------------------------------ 속도 프로파일
    def _build(self, pieces):
        sp = self.sp
        S, D, V, T, segs = [], [], [], [], []
        t_off, length = 0.0, 0.0
        for k, (sts, u, d, ds) in enumerate(pieces):
            n = len(sts)
            vmax = sp['v_max_forward_m_s'] if d > 0 else sp['v_max_reverse_m_s']
            lim = np.full(n, vmax)
            for i in range(n - 1):
                du = abs(u[i + 1] - u[i]) / ds if i + 1 < len(u) else 0.0
                if du > 1e-9:
                    lim[i] = min(lim[i], max(sp['v_min_m_s'], sp['steer_rate_max_rad_s'] / du))
            lim[0] = lim[-1] = 0.0
            v = lim.copy()
            a = sp['a_max_m_s2']
            for i in range(1, n):
                v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * a * ds))
            for i in range(n - 2, -1, -1):
                v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a * ds))
            t = np.zeros(n)
            for i in range(1, n):
                t[i] = t[i - 1] + 2 * ds / max(v[i] + v[i - 1], 2 * sp['v_min_m_s'] * 0.5)
            i0 = sum(len(x) for x in V)
            S.append(sts)
            D.append(u[:n])
            V.append(d * v)
            T.append(t + t_off)
            segs.append({'i0': i0, 'i1': i0 + n - 1, 'dir': int(d)})
            t_off += t[-1] + sp['cusp_dwell_s']
            length += ds * (n - 1)
        traj = RefTrajectory()
        traj.states = np.vstack(S)
        traj.states[:, 2] = [wrap(a) for a in traj.states[:, 2]]
        traj.states[:, 3] = [wrap(a) for a in traj.states[:, 3]]
        traj.delta = np.concatenate(D)
        traj.v = np.concatenate([np.asarray(x, dtype=float) for x in V])
        traj.t = np.concatenate(T)
        traj.segments = segs
        traj.length = length
        return traj
