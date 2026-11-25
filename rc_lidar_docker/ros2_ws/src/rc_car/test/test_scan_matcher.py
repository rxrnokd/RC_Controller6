"""scan_matcher 단위 테스트 (ROS 불필요):  python ros2_ws/src/rc_car/test/test_scan_matcher.py -v
가상 방(벽 + 기둥)에 대해 레이 캐스팅으로 스캔을 만들고, 알려진 이동을 복원하는지 확인한다.
※ 알고리즘 검증용 기하 테스트일 뿐 실제 라이다 성능을 보장하지 않음 (실차 확인 필요)."""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from rc_car import scan_matcher as sm  # noqa: E402

ROOM = [((-2.0, -1.5), (3.0, -1.5)), ((3.0, -1.5), (3.0, 2.0)), ((3.0, 2.0), (-2.0, 2.0)),
        ((-2.0, 2.0), (-2.0, -1.5)), ((0.8, 0.5), (1.2, 0.5)), ((1.2, 0.5), (1.2, 0.9)),
        ((1.2, 0.9), (0.8, 0.9)), ((0.8, 0.9), (0.8, 0.5))]
CORRIDOR = [((-5.0, -0.5), (5.0, -0.5)), ((-5.0, 0.5), (5.0, 0.5))]


def raycast(walls, pose, n=500, noise=0.0, rng=None):
    x, y, th = pose
    a = -math.pi + 2 * math.pi * np.arange(n) / n
    r = np.full(n, np.inf)
    dx, dy = np.cos(a + th), np.sin(a + th)
    for (x1, y1), (x2, y2) in walls:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            t = ((x1 - x) * ey - (y1 - y) * ex) / den
            u = ((x1 - x) * dy - (y1 - y) * dx) / den
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        r = np.where(hit & (t < r), t, r)
    if noise and rng is not None:
        r = r + rng.normal(0, noise, n)
    return r, a


def scan_pts(walls, pose, **kw):
    r, a = raycast(walls, pose, **kw)
    ok, r, a = sm.beam_mask(len(r), a[0], a[1] - a[0], r, 0.05, 8.0)
    return sm.to_base_points(r[ok], a[ok], (0.0, 0.0, 0.0))


class ScanMatcherTest(unittest.TestCase):
    def test_compose_inverse(self):
        a = (0.3, -0.2, 0.7)
        b = sm.compose(a, sm.inverse(a))
        assert max(abs(v) for v in b) < 1e-12

    def test_sector_mask(self):
        n = 360
        a0, inc = -math.pi, 2 * math.pi / n
        ok, _, a = sm.beam_mask(n, a0, inc, np.ones(n), 0.05, 8.0, [(math.radians(150), math.radians(-150))])
        rear = np.abs(np.degrees(np.arctan2(np.sin(a), np.cos(a)))) > 151
        assert not ok[rear].any() and ok[np.abs(np.degrees(a)) < 140].all()

    def test_icp_recovers_motion(self):
        p = dict(sm.DEFAULTS)
        ref = sm.Reference(sm.voxel_downsample(scan_pts(ROOM, (0, 0, 0)), p['voxel_m']))
        true = (0.08, -0.03, 0.06)
        res = sm.icp(ref, sm.voxel_downsample(scan_pts(ROOM, true), p['voxel_m']), (0, 0, 0), p)
        assert res['ok'], res
        assert abs(res['pose'][0] - true[0]) < 0.01 and abs(res['pose'][1] - true[1]) < 0.01
        assert abs(res['pose'][2] - true[2]) < 0.01

    def test_odometry_drive_and_reverse(self):
        rng = np.random.default_rng(0)
        lo = sm.LaserOdometry()
        pose, t = (0.0, 0.0, 0.0), 0.0
        # 전진 + 좌회전 1.0 m, 이후 후진 0.6 m (0.1 s 간격, 0.4 m/s)
        for k in range(40):
            v = 0.4 if k < 25 else -0.4
            w = 0.25 if k < 25 else 0.0
            pose = sm.compose(pose, (v * 0.1, 0.0, w * 0.1))
            t += 0.1
            res = lo.update(scan_pts(ROOM, pose, noise=0.005, rng=rng), t)
            assert res['ok'], (k, res)
        err = math.hypot(lo.pose[0] - pose[0], lo.pose[1] - pose[1])
        assert err < 0.05, (lo.pose, pose)
        assert abs(sm.wrap(lo.pose[2] - pose[2])) < 0.03
        assert lo.v < -0.3                       # 후진 속도는 음수
        assert lo.resets == 0

    def test_corridor_flags_degenerate(self):
        lo = sm.LaserOdometry()
        lo.update(scan_pts(CORRIDOR, (0, 0, 0)), 0.0)
        res = lo.update(scan_pts(CORRIDOR, (0.05, 0, 0)), 0.1)
        assert res['degenerate'] and res['status'] == 'DEGENERATE'

    def test_no_scene_is_lost_not_invented(self):
        lo = sm.LaserOdometry()
        lo.update(scan_pts(ROOM, (0, 0, 0)), 0.0)
        before = lo.pose
        res = lo.update(np.zeros((0, 2)), 0.1)
        assert not res['ok'] and lo.pose == before


if __name__ == '__main__':
    unittest.main()
