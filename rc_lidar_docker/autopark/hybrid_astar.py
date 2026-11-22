"""① Trailer-aware Hybrid A* — 견인차+트레일러 운동학을 그대로 적분하는 모션 프리미티브로 4D(x, y, th, beta) 탐색.

* 트레일러 차축 목표 자세 + 연결각 0 을 목표로 하며, 기본적으로 마지막 구간은 후진(트레일러 후진 주차).
* 전진 구간(위치 수정)도 허용, 전/후진 전환·조향·연결각에 비용.
* 매 substep 마다 견인차/트레일러 충돌영역(원 근사)과 연결각 한계(Jackknife 방지) 검사.
* 휴리스틱: 장애물을 반영한 2D Dijkstra(트레일러 차축 위치 → 목표) + 방향 오차.
실패 시 PlanResult.reason 에 원인 코드, message 에 설명.
"""
import heapq
import math
import time

import numpy as np

from .model import Footprint, roll_arc, trailer_pose, wrap

REASONS = {
    'START_IN_COLLISION': '시작 자세가 장애물/지도 밖과 겹침 (또는 미탐색 영역)',
    'GOAL_IN_COLLISION': '목표 자세가 장애물/지도 밖과 겹침',
    'START_HITCH_LIMIT': '현재 연결각이 hitch_angle_max 를 초과',
    'NO_PATH': '탐색 공간 내 경로 없음 (공간 부족 또는 목표 도달 불가)',
    'EXPANSION_LIMIT': 'max_expansions 초과',
    'TIMEOUT': 'time_limit_s 초과',
    'CANCELLED': '사용자 취소',
}


class PlanResult:
    def __init__(self, ok, reason='', states=None, steer=None, dirs=None, stats=None, message=''):
        self.ok = ok
        self.reason = reason
        self.message = message or REASONS.get(reason, '')
        self.states = states or []       # [(x, y, th, beta)], states[0] = 시작
        self.steer = steer or []          # states[i] 에 도달할 때 사용한 조향각 (i=0 은 첫 이동의 값)
        self.dirs = dirs or []            # +1 전진 / -1 후진 (위와 동일 규칙)
        self.stats = stats or {}

    def segments(self):
        """[(i0, i1, dir)] — states[i0..i1] 이 같은 방향으로 이어진 구간 (i0 는 이전 구간 끝점과 공유)."""
        segs = []
        if len(self.states) < 2:
            return segs
        i0, d = 0, self.dirs[1]
        for i in range(2, len(self.states)):
            if self.dirs[i] != d:
                segs.append((i0, i - 1, d))
                i0, d = i - 1, self.dirs[i]
        segs.append((i0, len(self.states) - 1, d))
        return segs


class HybridAStar:
    def __init__(self, geom, grid, cfg):
        self.g = geom
        self.grid = grid
        self.c = cfg['hybrid_astar']
        self.margin = cfg['map']['safety_margin_m']
        self.fp = Footprint(geom)
        n = max(2, int(self.c['steer_samples']))
        self.steers = [geom.steer_max * (2.0 * i / (n - 1) - 1.0) for i in range(n)]

    # ------------------------------------------------------------------
    def _valid(self, s):
        return abs(s[3]) <= self.g.hitch_max and not self.grid.collides(s, self.fp, self.margin)

    def _goal_reached(self, s, goal_tr, d):
        c = self.c
        if c.get('final_direction', 'reverse') == 'reverse' and d != -1:
            return False
        tx, ty, th2 = trailer_pose(s, self.g)
        return (math.hypot(tx - goal_tr[0], ty - goal_tr[1]) <= c['goal_tol_xy_m']
                and abs(wrap(th2 - goal_tr[2])) <= c['goal_tol_yaw_rad']
                and abs(s[3]) <= c['goal_tol_hitch_rad'])

    def _heuristic_grid(self, box, goal_xy):
        """장애물 반영 8방향 Dijkstra (트레일러 폭 절반 + 여유로 팽창한 점 로봇)."""
        res = self.c['xy_resolution_m']
        x0, y0, x1, y1 = box
        nx, ny = int((x1 - x0) / res) + 1, int((y1 - y0) / res) + 1
        clear = min(self.g.car_width, self.g.trailer_width) / 2.0 + self.margin
        free = np.zeros((ny, nx), dtype=bool)
        for iy in range(ny):
            y = y0 + (iy + 0.5) * res
            for ix in range(nx):
                free[iy, ix] = self.grid.dist_at(x0 + (ix + 0.5) * res, y) >= clear
        dist = np.full((ny, nx), np.inf)
        gx, gy = int((goal_xy[0] - x0) / res), int((goal_xy[1] - y0) / res)
        if not (0 <= gx < nx and 0 <= gy < ny):
            return dist, res
        dist[gy, gx] = 0.0
        pq = [(0.0, gx, gy)]
        nb = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
        while pq:
            d, x, y = heapq.heappop(pq)
            if d > dist[y, x]:
                continue
            for dx, dy, w in nb:
                xx, yy = x + dx, y + dy
                if 0 <= xx < nx and 0 <= yy < ny and free[yy, xx]:
                    nd = d + w * res
                    if nd < dist[yy, xx]:
                        dist[yy, xx] = nd
                        heapq.heappush(pq, (nd, xx, yy))
        return dist, res

    # ------------------------------------------------------------------
    def plan(self, start, goal, cancel=None):
        t0 = time.monotonic()
        c, g = self.c, self.g
        start, goal = tuple(start), tuple(goal)
        if abs(start[3]) > g.hitch_max:
            return PlanResult(False, 'START_HITCH_LIMIT')
        if self.grid.collides(start, self.fp, self.margin):
            return PlanResult(False, 'START_IN_COLLISION')
        if self.grid.collides(goal, self.fp, self.margin):
            return PlanResult(False, 'GOAL_IN_COLLISION')

        goal_tr = trailer_pose(goal, g)
        st_tr = trailer_pose(start, g)
        m = c['search_margin_m']
        xs = [start[0], goal[0], st_tr[0], goal_tr[0]]
        ys = [start[1], goal[1], st_tr[1], goal_tr[1]]
        box = (min(xs) - m, min(ys) - m, max(xs) + m, max(ys) + m)
        hdist, hres = self._heuristic_grid(box, goal_tr[:2])
        hny, hnx = hdist.shape

        def heur(s):
            tx, ty, th2 = trailer_pose(s, g)
            ix, iy = int((tx - box[0]) / hres), int((ty - box[1]) / hres)
            eu = math.hypot(tx - goal_tr[0], ty - goal_tr[1])
            d = hdist[iy, ix] if (0 <= ix < hnx and 0 <= iy < hny) else math.inf
            if not math.isfinite(d):
                d = 2.0 * eu + 1.0
            return max(d, eu) + c['heading_weight'] * abs(wrap(th2 - goal_tr[2])) + 0.2 * abs(s[3])

        res, ybin = c['xy_resolution_m'], 2 * math.pi / c['yaw_bins']
        hbin = c['hitch_resolution_rad']

        def key(s):
            return (int((s[0] - box[0]) / res), int((s[1] - box[1]) / res),
                    int(round(wrap(s[2]) / ybin)) % c['yaw_bins'], int(round(s[3] / hbin)))

        # 노드 저장: 병렬 리스트
        N_state, N_g, N_par, N_dir, N_steer, N_traj = [start], [0.0], [-1], [0], [0.0], [[]]
        best = {key(start): 0.0}
        hw = c.get('heuristic_weight', 1.0)
        openq = [(hw * heur(start), 0, 0)]
        counter = 1
        expansions = 0
        step, sub = c['step_m'], c['substep_m']
        dirs = (1, -1)
        while openq:
            _f, _c, idx = heapq.heappop(openq)
            s, gc = N_state[idx], N_g[idx]
            if best.get(key(s), math.inf) < gc - 1e-9:
                continue
            expansions += 1
            if expansions > c['max_expansions']:
                return PlanResult(False, 'EXPANSION_LIMIT', stats=self._stats(t0, expansions))
            if expansions % 200 == 0:
                if time.monotonic() - t0 > c['time_limit_s']:
                    return PlanResult(False, 'TIMEOUT', stats=self._stats(t0, expansions))
                if cancel is not None and cancel():
                    return PlanResult(False, 'CANCELLED', stats=self._stats(t0, expansions))
            if idx and self._goal_reached(s, goal_tr, N_dir[idx]):
                return self._reconstruct(idx, N_state, N_par, N_dir, N_steer, N_traj, t0, expansions)
            pdir, psteer = N_dir[idx], N_steer[idx]
            for d in dirs:
                for delta in self.steers:
                    traj = roll_arc(s, d, delta, step, sub, g)
                    ok = True
                    for st in traj:
                        if not (box[0] <= st[0] <= box[2] and box[1] <= st[1] <= box[3]) or not self._valid(st):
                            ok = False
                            break
                    if not ok:
                        continue
                    e = traj[-1]
                    cost = step * (c['reverse_cost'] if d < 0 else c['forward_cost'])
                    cost += c['steer_cost'] * abs(delta) / g.steer_max * step
                    cost += c['hitch_cost'] * abs(e[3]) * step
                    if pdir != 0:
                        cost += c['steer_change_cost'] * abs(delta - psteer) / g.steer_max
                        if d != pdir:
                            cost += c['switch_cost']
                    ng = gc + cost
                    k = key(e)
                    if best.get(k, math.inf) <= ng:
                        continue
                    best[k] = ng
                    N_state.append(e)
                    N_g.append(ng)
                    N_par.append(idx)
                    N_dir.append(d)
                    N_steer.append(delta)
                    N_traj.append(traj)
                    heapq.heappush(openq, (ng + hw * heur(e), counter, len(N_state) - 1))
                    counter += 1
        return PlanResult(False, 'NO_PATH', stats=self._stats(t0, expansions))

    @staticmethod
    def _stats(t0, expansions):
        return {'time_s': round(time.monotonic() - t0, 3), 'expansions': expansions}

    def _reconstruct(self, idx, N_state, N_par, N_dir, N_steer, N_traj, t0, expansions):
        chain = []
        while idx > 0:
            chain.append(idx)
            idx = N_par[idx]
        chain.reverse()
        states, steer, dirs = [N_state[0]], [N_steer[chain[0]]], [N_dir[chain[0]]]
        for i in chain:
            for st in N_traj[i]:
                states.append(st)
                steer.append(N_steer[i])
                dirs.append(N_dir[i])
        length = (len(states) - 1) * self.c['step_m'] / max(1, len(N_traj[chain[0]]))
        stats = self._stats(t0, expansions)
        stats['length_m'] = round(length, 3)
        r = PlanResult(True, '', states, steer, dirs, stats, message='경로 생성 성공')
        stats['segments'] = len(r.segments())
        return r
