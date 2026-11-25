"""라이다 스캔매칭 Odometry — DC 모터 엔코더 없이 라이다만으로 odom → base_link 를 추정.

  입력  /scan (sensor_msgs/LaserScan, sllidar C1)
  출력  /odom (nav_msgs/Odometry),  TF odom → base_link (스캔 시각 기준)
        /scan_filtered (자기 차체·트레일러 영역을 제외한 스캔 → SLAM Toolbox 입력)
        /rc/odom_status (std_msgs/String JSON: 정합 품질, 상태)

SLAM Toolbox 는 odom → base_link TF 가 있어야 동작하므로, 이 노드가 엔코더 odom 을 대신한다.
정합에 실패한 스캔은 자세를 갱신하지도, /odom·TF 를 발행하지도 않는다 (추정값으로 메우지 않음).
알고리즘 본체는 scan_matcher.py (ROS 비의존, 단위 테스트 있음).
"""
import math
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .common import dumps, load_config

try:
    import numpy as np
    from . import scan_matcher as sm
    _IMPORT_ERROR = None
except ImportError as e:   # numpy 없음 → 노드는 떠서 원인을 상태로 알린다
    np = sm = None
    _IMPORT_ERROR = str(e)


class OdometryNode(Node):
    def __init__(self):
        super().__init__('rc_odometry')
        self.declare_parameter('config_file', '')
        cfg = load_config(self.get_parameter('config_file').value or None)
        lc = cfg['lidar']
        oc = cfg.get('odometry') or {}
        self.mount = (float(lc.get('x', 0.0)), float(lc.get('y', 0.0)), float(lc.get('yaw', 0.0)))
        self.rmin = float(lc.get('mask_min_range_m', 0.12))
        self.rmax = float(oc.get('max_range_m', 8.0))
        self.sectors = [(math.radians(a), math.radians(b)) for a, b in (lc.get('mask_sectors_deg') or [])]
        self.publish_tf = bool(oc.get('publish_tf', True))
        self.lo = sm.LaserOdometry({k: oc.get(k) for k in sm.DEFAULTS}) if sm else None
        self.status = {'status': 'NO_SCAN', 'ok': False}
        self.proc_ms = None
        self.scan_rx_t = 0.0
        self.last_ok_t = 0.0
        self.pub = self.create_publisher(Odometry, '/odom', 20)
        self.pub_scan = self.create_publisher(LaserScan, '/scan_filtered', 5)
        self.pub_status = self.create_publisher(String, '/rc/odom_status', 10)
        self.tfb = TransformBroadcaster(self)
        # 처리 지연 시 오래된 스캔이 쌓이지 않도록 최신 1개만 유지
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos)
        self.create_timer(0.2, self._publish_status)
        if _IMPORT_ERROR:
            self.get_logger().error(f'numpy 불러오기 실패 → 라이다 odom 비활성: {_IMPORT_ERROR} '
                                    '(컨테이너에 python3-numpy 설치 필요)')
        elif sm.cKDTree is None:
            self.get_logger().warn('scipy 없음 → 최근접 탐색을 numpy 전수 비교로 수행 (느림). '
                                   '권장: apt install python3-scipy')
        self.get_logger().info(f'라이다 odom: mount={self.mount} 범위 {self.rmin}–{self.rmax} m, '
                               f'제외 구간(deg)={lc.get("mask_sectors_deg") or []}')

    def _on_scan(self, scan):
        t0 = time.monotonic()
        self.scan_rx_t = t0
        if sm is None:
            self.status = {'status': 'NO_NUMPY', 'ok': False, 'reason': _IMPORT_ERROR}
            return
        n = len(scan.ranges)
        keep, r, a = sm.beam_mask(n, scan.angle_min, scan.angle_increment, scan.ranges,
                                  max(self.rmin, scan.range_min), min(self.rmax, scan.range_max), self.sectors)
        # SLAM 용 필터 스캔 (제외 빔 = +inf: REP-117 '측정 범위 밖')
        fs = LaserScan()
        fs.header = scan.header
        fs.angle_min, fs.angle_max, fs.angle_increment = scan.angle_min, scan.angle_max, scan.angle_increment
        fs.time_increment, fs.scan_time = scan.time_increment, scan.scan_time
        fs.range_min, fs.range_max = scan.range_min, scan.range_max
        fr = np.where(keep, r, np.inf)
        fs.ranges = fr.astype(np.float32).tolist()
        fs.intensities = list(scan.intensities)
        self.pub_scan.publish(fs)

        pts = sm.to_base_points(r[keep], a[keep], self.mount)
        t = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        res = self.lo.update(pts, t)
        self.status = res
        self.proc_ms = (time.monotonic() - t0) * 1000.0
        if not res.get('ok'):
            return
        self.last_ok_t = time.monotonic()
        self._publish_odom(scan.header.stamp, res)

    def _publish_odom(self, stamp, res):
        x, y, th = self.lo.pose
        qz, qw = math.sin(th / 2.0), math.cos(th / 2.0)
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = self.lo.v
        od.twist.twist.angular.z = self.lo.w
        # 공분산: 정합 잔차 기반의 대략값. 퇴화(복도 등) 시 크게 표시
        rms = res.get('rms_m') or 0.02
        k = 100.0 if res.get('degenerate') else 1.0
        pc = [0.0] * 36
        pc[0] = pc[7] = max(1e-4, (rms * 2) ** 2) * k
        pc[14] = pc[21] = pc[28] = 1e6          # 2D: z, roll, pitch 미사용
        pc[35] = max(1e-4, (rms * 2) ** 2) * k
        od.pose.covariance = pc
        tc = [0.0] * 36
        tc[0], tc[7], tc[35] = 0.01 * k, 1e6, 0.02 * k
        tc[14] = tc[21] = tc[28] = 1e6
        od.twist.covariance = tc
        self.pub.publish(od)
        if self.publish_tf:
            tf = TransformStamped()
            tf.header = od.header
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tfb.sendTransform(tf)

    def _publish_status(self):
        now = time.monotonic()
        st = dict(self.status)
        if now - self.scan_rx_t > 1.0:
            st = {'status': 'NO_SCAN', 'ok': False}
        st.update({
            'source': 'lidar_scan_matching',
            'ok': bool(st.get('ok')) and now - self.last_ok_t < 0.5,
            'proc_ms': None if self.proc_ms is None else round(self.proc_ms, 1),
            'resets': self.lo.resets if self.lo else 0,
            'v': round(self.lo.v, 3) if self.lo else None,
            'kdtree': bool(sm and sm.cKDTree is not None),
            'stamp': time.time(),
        })
        self.pub_status.publish(String(data=dumps(st)))


def main():
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
