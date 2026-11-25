"""수학/알고리즘 단위 테스트 (차량 시뮬레이션 루프 없음).  실행: python -m unittest autopark.test_autopark -v
테스트용 치수는 이 파일 안에서만 쓰는 가정값이며 config.yaml 의 실측값을 대신하지 않는다."""
import copy
import math
import time
import unittest

import numpy as np

from . import load_config, plan_parking
from .__main__ import FeedbackMonitor
from .model import (Footprint, Geometry, GridMap, geometry_from_config, roll_arc, state_from_trailer_goal, step,
                    trailer_pose, wrap)
from .tracking_mpc import TrackingMPC

G = Geometry(wheelbase=0.17, steer_max=0.45, car_length=0.27, car_width=0.197, car_rear=0.05,
             hitch_offset=0.06, trailer_length=0.25, trailer_front=0.12, trailer_rear=0.08,
             trailer_width=0.18, hitch_max=1.0)


def test_cfg():
    cfg = load_config()
    cfg = copy.deepcopy(cfg)
    cfg['vehicle'].update({'steer_max_rad': G.steer_max})
    cfg['trailer'].update({'hitch_offset_m': G.hitch_offset, 'length_m': G.trailer_length,
                           'front_m': G.trailer_front, 'rear_m': G.trailer_rear,
                           'width_m': G.trailer_width, 'hitch_angle_max_rad': G.hitch_max})
    return cfg


class TestKinematics(unittest.TestCase):
    def test_straight(self):
        s = (0.0, 0.0, 0.0, 0.0)
        for _ in range(100):
            s = step(s, 1.0, 0.0, 0.01, G)
        self.assertAlmostEqual(s[0], 1.0, places=9)
        self.assertAlmostEqual(s[3], 0.0, places=12)

    def test_turn_rate(self):
        d = 0.3
        s = roll_arc((0, 0, 0, 0), 1, d, 0.5, 0.01, G)[-1]
        self.assertAlmostEqual(s[2], 0.5 * math.tan(d) / G.wheelbase, places=6)

    def test_reversible(self):
        s0 = (1.0, 2.0, 0.3, 0.2)
        s1 = roll_arc(s0, 1, 0.25, 0.6, 0.005, G)[-1]
        s2 = roll_arc(s1, -1, 0.25, 0.6, 0.005, G)[-1]
        for a, b in zip(s0, s2):
            self.assertAlmostEqual(a, b, places=6)

    def test_forward_stable_reverse_unstable(self):
        s = (0, 0, 0, 0.3)
        f = roll_arc(s, 1, 0.0, 1.0, 0.01, G)[-1]
        r = roll_arc(s, -1, 0.0, 0.3, 0.01, G)[-1]
        self.assertLess(abs(f[3]), 0.3)     # 전진: 연결각 감소
        self.assertGreater(abs(r[3]), 0.3)  # 후진: 연결각 증가 (불안정 → 폐루프 필요)

    def test_goal_roundtrip(self):
        s = state_from_trailer_goal(1.0, 2.0, 0.7, G)
        tx, ty, th2 = trailer_pose(s, G)
        self.assertAlmostEqual(tx, 1.0)
        self.assertAlmostEqual(ty, 2.0)
        self.assertAlmostEqual(th2, 0.7)
        self.assertEqual(s[3], 0.0)

    def test_footprint_covers_corners(self):
        fp = Footprint(G)
        s = (0.0, 0.0, 0.0, 0.0)
        xs, ys, rs = fp.circles(s)
        car, tr = fp.outline(s)
        for corners, idx in ((car, range(0, len(fp.car_offs))),
                             (tr, range(len(fp.car_offs), len(xs)))):
            for cx, cy in corners:
                self.assertTrue(any(math.hypot(cx - xs[i], cy - ys[i]) <= rs[i] + 1e-3 for i in idx))


class TestMap(unittest.TestCase):
    def test_collision(self):
        g = GridMap.empty(3, 3, 0.05)
        fp = Footprint(G)
        self.assertFalse(g.collides((1.5, 1.5, 0, 0), fp, 0.05))
        self.assertTrue(g.collides((0.1, 1.5, 0, 0), fp, 0.05))
        g.occ[30, 40] = True
        g2 = GridMap(g.occ, 0.05, 0, 0)
        self.assertTrue(g2.collides((1.95, 1.5, 0, 0), fp, 0.0))

    def test_config_nulls_refused(self):
        geom, missing = geometry_from_config(load_config())
        self.assertIsNone(geom)
        self.assertIn('trailer.length_m', missing)


def bay_map():
    """폭 5 m x 4 m, 아래쪽에 트레일러 주차칸(폭 0.45 m, 깊이 0.6 m)."""
    g = GridMap.empty(5.0, 4.0, 0.05)
    occ = g.occ.copy()
    res = 0.05
    for x in np.arange(0, 5, res):
        if not (2.275 <= x <= 2.725):
            occ[int(0.6 / res), int(x / res)] = True
    for y in np.arange(0, 0.6, res):
        occ[int(y / res), int(2.25 / res)] = True
        occ[int(y / res), int(2.75 / res)] = True
    return GridMap(occ, res, 0, 0)


class TestPlanning(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = test_cfg()
        cls.geom, _ = geometry_from_config(cls.cfg)

    def _check_traj(self, grid, traj):
        fp = Footprint(self.geom)
        for s in traj.states:
            self.assertLessEqual(abs(s[3]), self.geom.hitch_max + 1e-9)
            self.assertFalse(grid.collides(tuple(s), fp, self.cfg['map']['safety_margin_m']))
        self.assertTrue(np.all(np.abs(traj.delta) <= self.geom.steer_max + 1e-9))
        self.assertEqual(traj.segments[-1]['dir'], -1)       # 마지막은 후진
        for sg in traj.segments:
            v = traj.v[sg['i0']:sg['i1'] + 1]
            self.assertEqual(v[0], 0.0)
            self.assertEqual(v[-1], 0.0)
            self.assertTrue(np.all(v * sg['dir'] >= 0))

    def test_straight_reverse(self):
        grid = GridMap.empty(5, 3, 0.05)
        start = (3.5, 1.5, 0.0, 0.0)
        res, traj, goal = plan_parking(self.cfg, self.geom, grid, start, (1.6, 1.5, 0.0))
        self.assertTrue(res.ok, res.reason)
        self._check_traj(grid, traj)
        tx, ty, th2 = trailer_pose(tuple(traj.states[-1]), self.geom)
        self.assertLess(math.hypot(tx - 1.6, ty - 1.5), self.cfg['hybrid_astar']['goal_tol_xy_m'])

    def test_bay_reverse_parking(self):
        grid = bay_map()
        start = (1.2, 1.6, 0.0, 0.0)
        t0 = time.monotonic()
        res, traj, goal = plan_parking(self.cfg, self.geom, grid, start, (2.5, 0.25, math.pi / 2))
        print(f'\n  bay: {res.reason or "ok"} {res.stats} mpc={traj.summary() if traj else None} '
              f'{time.monotonic() - t0:.1f}s')
        self.assertTrue(res.ok, res.reason)
        self._check_traj(grid, traj)

    def test_failure_reasons(self):
        grid = bay_map()
        r, _, _ = plan_parking(self.cfg, self.geom, grid, (0.05, 1.6, 0, 0), (2.5, 0.25, math.pi / 2))
        self.assertEqual(r.reason, 'START_IN_COLLISION')
        r, _, _ = plan_parking(self.cfg, self.geom, grid, (1.2, 1.6, 0, 1.3), (2.5, 0.25, math.pi / 2))
        self.assertEqual(r.reason, 'START_HITCH_LIMIT')
        r, _, _ = plan_parking(self.cfg, self.geom, grid, (1.2, 1.6, 0, 0), (2.5, 0.25, 0.0))
        self.assertEqual(r.reason, 'GOAL_IN_COLLISION')


class TestTracking(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = test_cfg()
        cls.geom, _ = geometry_from_config(cls.cfg)
        grid = GridMap.empty(5, 3, 0.05)
        _, cls.traj, _ = plan_parking(cls.cfg, cls.geom, grid, (3.5, 1.5, 0.0, 0.0), (1.6, 1.5, 0.0))

    def fb(self, s, v=0.05, us=None):
        return {'x': s[0], 'y': s[1], 'th': s[2], 'beta': s[3], 'v': v, 'us_min': us, 't': time.monotonic()}

    def test_jacobian(self):
        tm = TrackingMPC(self.geom, self.cfg)
        s, v, d, dt = (0.0, 0.0, 0.2, 0.3), -0.08, 0.1, 0.1
        A, B = tm.linearize(s, v, d, dt)
        e = np.array([1e-4, -2e-4, 1e-4, 2e-4])
        lin = np.array(step(s, v, d, dt, self.geom)) + A @ e
        non = np.array(step(tuple(np.array(s) + e), v, d, dt, self.geom))
        self.assertLess(np.max(np.abs(lin - non)), 1e-7)

    def test_on_reference(self):
        tm = TrackingMPC(self.geom, self.cfg)
        tm.reset(self.traj)
        i = self.traj.segments[0]['i0'] + 3
        out = tm.step(self.fb(tuple(self.traj.states[i])))
        self.assertEqual(out.status, 'TRACKING')
        self.assertLess(abs(out.errors['lateral_m']), 1e-6)
        self.assertEqual(math.copysign(1, out.v), self.traj.segments[0]['dir'])
        self.assertLessEqual(abs(out.steer), self.geom.steer_max)

    def test_safety(self):
        tm = TrackingMPC(self.geom, self.cfg)
        tm.reset(self.traj)
        s = list(self.traj.states[3])
        self.assertEqual(tm.step(self.fb((s[0], s[1], s[2], 1.2))).status, 'STOP')       # 연결각 한계
        self.assertEqual(tm.step(self.fb(tuple(s), us=0.03)).status, 'STOP')             # 초음파
        self.assertEqual(tm.step(self.fb((s[0], s[1] + 0.3, s[2], s[3]))).status, 'REPLAN')  # 이탈

    def test_steer_rate_limit(self):
        tm = TrackingMPC(self.geom, self.cfg)
        tm.reset(self.traj, steer_now=-self.geom.steer_max)
        s = list(self.traj.states[3])
        out = tm.step(self.fb((s[0], s[1] + 0.05, s[2], s[3])))
        lim = self.cfg['speed']['steer_rate_max_rad_s'] * self.cfg['tracking_mpc']['dt_s']
        self.assertLessEqual(abs(out.steer + self.geom.steer_max), lim + 1e-6)

    def test_parked(self):
        tm = TrackingMPC(self.geom, self.cfg)
        tm.reset(self.traj)
        tm.seg = len(self.traj.segments) - 1
        tm.idx = self.traj.segments[-1]['i1'] - 1
        out = tm.step(self.fb(tuple(self.traj.states[-1]), v=0.0))
        self.assertEqual(out.status, 'PARKED', out.reason)


class TestFeedback(unittest.TestCase):
    def test_feedback_gating(self):
        cfg = test_cfg()
        st = {'t': 100.0, 'pose': {'frame': 'map', 'x': 1, 'y': 1, 'yaw': 0, 'age_s': 0.05},
              'vehicle': {'stamp': 99.95, 'encoders_used': False, 'encoders_ok': False, 'speed_m_s': None,
                          'trailer': {'enabled': True, 'angle_rad': 0.1}, 'steering': {'calibrated': True},
                          'ultrasonic': []},
              'odom': None, 'slam': {'odom_ok': False, 'odom_status': {'status': 'LOST', 'ok': False}},
              'control': {'autopark': {'enabled': True}, 'estop': False}}
        mon = FeedbackMonitor(cfg)
        rep = mon.evaluate(st, 10.0, 10.05)
        self.assertFalse(rep['motion_ok'])
        self.assertTrue(rep['planning_start_ok'])      # 계획·검증은 가능
        self.assertNotIn('v', rep['values'])           # 속도 대체값 생성 금지
        # 라이다 odom 정상 → 속도 실측은 생기지만, 엔코더가 없어 차량 속도 폐루프 제어가 불가 → 구동 차단 유지
        st['odom'] = {'v': 0.0}
        st['slam'] = {'odom_ok': True, 'odom_status': {'status': 'OK', 'ok': True}}
        rep = mon.evaluate(st, 10.0, 10.05)
        self.assertEqual(rep['values']['v'], 0.0)
        self.assertTrue(rep['checks']['speed']['ok'])
        self.assertFalse(rep['checks']['speed_control']['ok'])
        self.assertFalse(rep['motion_ok'])
        # 퇴화(복도형) 상태의 라이다 odom 속도는 쓰지 않음
        st['slam']['odom_status']['status'] = 'DEGENERATE'
        self.assertNotIn('v', mon.evaluate(st, 10.0, 10.05)['values'])
        st['slam']['odom_status']['status'] = 'OK'
        # (엔코더 사용 구성) 모든 조건 충족 시에만 허용, map 이 아닌 odom 위치로는 불가
        st['vehicle'].update({'encoders_used': True, 'encoders_ok': True, 'speed_m_s': 0.0})
        self.assertTrue(mon.evaluate(st, 10.0, 10.05)['motion_ok'])
        st['pose']['frame'] = 'odom'
        self.assertFalse(mon.evaluate(st, 10.0, 10.05)['motion_ok'])


if __name__ == '__main__':
    unittest.main()
