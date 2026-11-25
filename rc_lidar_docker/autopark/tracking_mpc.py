"""③ Tracking MPC — 실제 피드백(SLAM 위치·방향, 실측 속도(라이다 odom), MT6701 연결각)으로 기준 궤적 추종.

선형 시변(LTV) MPC: 기준 궤적 주변에서 RK4 이산 모델을 수치 선형화
   e_{k+1} = A_k e_k + B_k u_k,   e = x - x_ref,  u = δ - δ_ref
   min Σ e_kᵀQ e_k + r·Σu_k² + r_rate·Σ(δ_k - δ_{k-1})²   s.t.  |δ_k| ≤ δmax, |δ_0 - δ_prev| ≤ rate·dt
→ 경계 제약 최소자승(scipy lsq_linear)으로 풀이. 출력 = 목표 속도 v[m/s], 목표 조향각 δ[rad].

안전/상태 판정 (우선순위): 피드백 무효 → STOP, 연결각 한계 → STOP, 초음파 근접 → STOP,
경로 이탈(횡/방향/연결각 오차) → REPLAN, 구간 끝 → 정지 후 다음 구간, 마지막 구간 도달 → PARKED.
"""
import math

import numpy as np
from scipy.optimize import lsq_linear

from .model import step as fstep, trailer_pose, wrap


class TrackOutput:
    def __init__(self, status, v=0.0, steer=0.0, reason='', errors=None, info=None):
        self.status = status          # TRACKING | SEGMENT_STOP | PARKED | REPLAN | STOP
        self.v = v
        self.steer = steer
        self.reason = reason
        self.errors = errors or {}
        self.info = info or {}


class TrackingMPC:
    def __init__(self, geom, cfg):
        self.g = geom
        self.c = cfg['tracking_mpc']
        self.sp = cfg['speed']
        self.traj = None

    def reset(self, traj, steer_now=0.0):
        self.traj = traj
        self.seg = 0
        self.idx = traj.segments[0]['i0']
        self.prev_steer = float(steer_now)
        self.stopped_since = None

    # ------------------------------------------------------------------ 기준 궤적 보조
    def _nearest(self, x, y):
        sg = self.traj.segments[self.seg]
        hi = min(sg['i1'], self.idx + self.c['search_window'])
        pts = self.traj.states[self.idx:hi + 1, :2]
        k = int(np.argmin((pts[:, 0] - x) ** 2 + (pts[:, 1] - y) ** 2))
        self.idx = self.idx + k
        return self.idx

    def _ref_at(self, t):
        tr, sg = self.traj, self.traj.segments[self.seg]
        i0, i1 = sg['i0'], sg['i1']
        ts = tr.t[i0:i1 + 1]
        t = min(max(t, ts[0]), ts[-1])
        j = int(np.searchsorted(ts, t, side='right')) - 1
        j = min(max(j, 0), len(ts) - 2) if len(ts) > 1 else 0
        if len(ts) == 1:
            return tr.states[i0].copy(), tr.delta[i0], tr.v[i0]
        a = 0.0 if ts[j + 1] == ts[j] else (t - ts[j]) / (ts[j + 1] - ts[j])
        s0, s1 = tr.states[i0 + j], tr.states[i0 + j + 1]
        s = s0 + a * (s1 - s0)
        s[2] = wrap(s0[2] + a * wrap(s1[2] - s0[2]))
        s[3] = wrap(s0[3] + a * wrap(s1[3] - s0[3]))
        return s, tr.delta[i0 + j] * (1 - a) + tr.delta[i0 + j + 1] * a, tr.v[i0 + j] * (1 - a) + tr.v[i0 + j + 1] * a

    def linearize(self, s, v, delta, dt):
        """A = ∂f/∂x (4x4), B = ∂f/∂δ (4,) — 중앙차분."""
        A = np.zeros((4, 4))
        h = 1e-5
        for i in range(4):
            sp, sm = list(s), list(s)
            sp[i] += h
            sm[i] -= h
            fp, fm = fstep(sp, v, delta, dt, self.g), fstep(sm, v, delta, dt, self.g)
            A[:, i] = [(fp[k] - fm[k]) if k < 2 else wrap(fp[k] - fm[k]) for k in range(4)]
        A /= 2 * h
        fp, fm = fstep(s, v, delta + h, dt, self.g), fstep(s, v, delta - h, dt, self.g)
        B = np.array([(fp[k] - fm[k]) if k < 2 else wrap(fp[k] - fm[k]) for k in range(4)]) / (2 * h)
        return A, B

    # ------------------------------------------------------------------ MPC 풀이
    def solve(self, s_now, t_ref, direction):
        c, g = self.c, self.g
        N, dt = c['horizon'], c['dt_s']
        refs = [self._ref_at(t_ref + k * dt) for k in range(N + 1)]
        e0 = np.array(s_now, dtype=float) - refs[0][0]
        e0[2], e0[3] = wrap(e0[2]), wrap(e0[3])
        vmin = self.sp['v_min_m_s']
        As, Bs = [], []
        for k in range(N):
            r, dr, vr = refs[k]
            vlin = direction * max(abs(vr), vmin)
            A, B = self.linearize(tuple(r), vlin, dr, dt)
            As.append(A)
            Bs.append(B)
        F = np.zeros((4 * N, 4))
        G = np.zeros((4 * N, N))
        Phi = np.eye(4)
        for k in range(N):                     # e_{k+1} 블록
            Phi = As[k] @ Phi
            F[4 * k:4 * k + 4] = Phi
            for j in range(k + 1):
                M = Bs[j]
                for m in range(j + 1, k + 1):
                    M = As[m] @ M
                G[4 * k:4 * k + 4, j] = M
        w = np.sqrt(np.tile([c['q_xy'], c['q_xy'], c['q_yaw'], c['q_hitch']], N))
        dref = np.array([refs[k][1] for k in range(N)])
        D = np.eye(N) - np.eye(N, k=-1)
        cvec = np.diff(np.concatenate([[self.prev_steer], dref]))
        rows = [w[:, None] * G, math.sqrt(c['r_steer']) * np.eye(N), math.sqrt(c['r_steer_rate']) * D]
        rhs = [-w * (F @ e0), np.zeros(N), -math.sqrt(c['r_steer_rate']) * cvec]
        Amat, bvec = np.vstack(rows), np.concatenate(rhs)
        lb, ub = -g.steer_max - dref, g.steer_max - dref
        rate = self.sp['steer_rate_max_rad_s'] * dt
        lb0 = max(lb[0], self.prev_steer - rate - dref[0])
        ub0 = min(ub[0], self.prev_steer + rate - dref[0])
        if lb0 > ub0:
            lb0 = ub0 = min(max(self.prev_steer - dref[0], lb[0]), ub[0])
        lb[0], ub[0] = lb0, ub0 + 1e-12
        sol = lsq_linear(Amat, bvec, bounds=(lb, ub))
        u = sol.x
        steer = float(np.clip(dref[0] + u[0], -g.steer_max, g.steer_max))
        pred = (F @ e0 + G @ u).reshape(N, 4)
        pts = [[round(float(refs[k + 1][0][0] + pred[k, 0]), 3), round(float(refs[k + 1][0][1] + pred[k, 1]), 3)]
               for k in range(N)]
        return steer, pts

    # ------------------------------------------------------------------ 1 제어 주기
    def step(self, fb):
        """fb: {'x','y','th','beta','v'(실측 m/s: 라이다 odom 또는 엔코더), 'us_min'(m 또는 None)} — 모두 실제 측정값이어야 함."""
        c, g, tr = self.c, self.g, self.traj
        s = (fb['x'], fb['y'], fb['th'], fb['beta'])
        if abs(s[3]) > g.hitch_max:
            return TrackOutput('STOP', reason=f'연결각 {math.degrees(s[3]):.1f}° > 한계 {math.degrees(g.hitch_max):.1f}°')
        if fb.get('us_min') is not None and fb['us_min'] < c['ultrasonic_stop_m']:
            return TrackOutput('STOP', reason=f'초음파 근접 {fb["us_min"] * 100:.1f} cm')
        sg = tr.segments[self.seg]
        d = sg['dir']
        i = self._nearest(s[0], s[1])
        r = tr.states[i]
        e_lat = -math.sin(r[2]) * (s[0] - r[0]) + math.cos(r[2]) * (s[1] - r[1])
        e_head, e_hitch = wrap(s[2] - r[2]), wrap(s[3] - r[3])
        txy, rtxy = trailer_pose(s, g), trailer_pose(tuple(r), g)
        e_tlat = -math.sin(rtxy[2]) * (txy[0] - rtxy[0]) + math.cos(rtxy[2]) * (txy[1] - rtxy[1])
        errors = {'lateral_m': round(e_lat, 4), 'heading_rad': round(e_head, 4), 'hitch_rad': round(e_hitch, 4),
                  'trailer_lateral_m': round(e_tlat, 4)}
        info = {'segment': self.seg, 'segments': len(tr.segments), 'index': i, 'direction': d,
                'progress': round(float(tr.t[i] / tr.t[-1]) if tr.t[-1] > 0 else 1.0, 3),
                'ref': [round(float(r[0]), 3), round(float(r[1]), 3)]}
        if abs(e_lat) > c['max_lateral_error_m'] or abs(e_head) > c['max_heading_error_rad'] or \
                abs(e_hitch) > c['max_hitch_error_rad']:
            return TrackOutput('REPLAN', reason='경로 이탈', errors=errors, info=info)

        v_meas = fb['v']
        end = tr.states[sg['i1']]
        tdir = (d * math.cos(end[2]), d * math.sin(end[2]))
        remaining = (end[0] - s[0]) * tdir[0] + (end[1] - s[1]) * tdir[1]
        info['remaining_m'] = round(remaining, 3)
        if i >= sg['i1'] - 3 and remaining <= c['segment_end_tol_m']:
            stopped = abs(v_meas) < c['stop_speed_m_s']
            if self.seg == len(tr.segments) - 1:
                gx, gy, gth = trailer_pose(tr.goal, g)
                exy, eth = math.hypot(txy[0] - gx, txy[1] - gy), abs(wrap(txy[2] - gth))
                info['goal_error'] = {'xy_m': round(exy, 3), 'yaw_rad': round(eth, 3), 'hitch_rad': round(abs(s[3]), 3)}
                ok = exy <= c['arrive_tol_xy_m'] and eth <= c['arrive_tol_yaw_rad'] and \
                    abs(s[3]) <= c['arrive_tol_hitch_rad']
                if not stopped:
                    return TrackOutput('SEGMENT_STOP', 0.0, self.prev_steer, '목표 도달 — 정지 중', errors, info)
                if ok:
                    return TrackOutput('PARKED', 0.0, self.prev_steer, '주차 완료', errors, info)
                return TrackOutput('REPLAN', reason='최종 자세 오차 초과', errors=errors, info=info)
            if stopped:
                self.stopped_since = self.stopped_since or fb.get('t', 0.0)
                if fb.get('t', 0.0) - self.stopped_since >= self.sp['cusp_dwell_s']:
                    self.seg += 1
                    self.idx = tr.segments[self.seg]['i0']
                    self.stopped_since = None
            return TrackOutput('SEGMENT_STOP', 0.0, self.prev_steer, '전/후진 전환점 정지', errors, info)
        self.stopped_since = None

        steer, pred = self.solve(s, float(tr.t[i]), d)
        self.prev_steer = steer
        _r, _dr, v_la = self._ref_at(float(tr.t[i]) + c['lookahead_s'])
        scale = max(0.3, 1.0 - abs(e_lat) / c['max_lateral_error_m'])
        vmag = max(self.sp['v_min_m_s'], abs(v_la)) * scale
        vmax = self.sp['v_max_forward_m_s'] if d > 0 else self.sp['v_max_reverse_m_s']
        info['predicted'] = pred
        return TrackOutput('TRACKING', d * min(vmag, vmax), steer, '', errors, info)
