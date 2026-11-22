"""엔코더(DC 모터 내장) + 서보 명령 조향각 기반 차량 운동학 Odometry.

  입력  /joint_states (뒷바퀴 위치 rad), /steering/angle_cmd (rad), /rc/vehicle_status (encoders_ok)
  출력  /odom (nav_msgs/Odometry, odom → base_link),  TF odom → base_link
  base_link 원점 = 뒷차축 중심(지면). 자전거 모델:
     ds = (dL + dR)/2,  dyaw = ds·tan(δ)/L  (yaw_source: steering)  또는  (dR − dL)/track (wheel_diff)
엔코더 데이터가 끊기거나 엔코더 고장이면 /odom·TF 발행을 중단한다 (SLAM 이 잘못된 odom 을 쓰지 않도록).
"""
import json
import math
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from tf2_ros import TransformBroadcaster

from .common import load_config


class OdometryNode(Node):
    def __init__(self):
        super().__init__('rc_odometry')
        self.declare_parameter('config_file', '')
        cfg = load_config(self.get_parameter('config_file').value or None)
        self.r = cfg['vehicle']['wheel_diameter_m'] / 2.0
        self.L = cfg['vehicle']['wheelbase_m']
        self.track = cfg['vehicle']['track_width_m']
        self.names = (cfg['motors']['left']['name'], cfg['motors']['right']['name'])
        oc = cfg['odometry']
        self.yaw_source = oc.get('yaw_source', 'steering')
        self.publish_tf = oc.get('publish_tf', True)
        self.timeout = oc.get('input_timeout_s', 0.3)
        self.x = self.y = self.yaw = 0.0
        self.prev = None
        self.prev_t = None
        self.steer = 0.0
        self.steer_t = 0.0
        self.enc_ok = False
        self.enc_ok_t = 0.0
        self.pub = self.create_publisher(Odometry, '/odom', 20)
        self.tfb = TransformBroadcaster(self)
        self.create_subscription(JointState, '/joint_states', self._on_js, 20)
        self.create_subscription(Float64, '/steering/angle_cmd', self._on_steer, 20)
        self.create_subscription(String, '/rc/vehicle_status', self._on_status, 10)
        self.get_logger().info(f'Odometry yaw_source={self.yaw_source} L={self.L} track={self.track}')

    def _on_steer(self, m):
        self.steer, self.steer_t = m.data, time.monotonic()

    def _on_status(self, m):
        try:
            self.enc_ok = bool(json.loads(m.data).get('encoders_ok'))
            self.enc_ok_t = time.monotonic()
        except ValueError:
            self.enc_ok = False

    def _on_js(self, js):
        try:
            iL, iR = js.name.index(self.names[0]), js.name.index(self.names[1])
        except ValueError:
            return
        now = time.monotonic()
        pos = (js.position[iL], js.position[iR])
        t = js.header.stamp.sec + js.header.stamp.nanosec * 1e-9
        healthy = self.enc_ok and now - self.enc_ok_t < 1.0
        if self.yaw_source == 'steering':
            healthy = healthy and now - self.steer_t < self.timeout
        if self.prev is None or not healthy or t - self.prev_t > self.timeout or t <= self.prev_t:
            self.prev, self.prev_t = pos, t     # 재시작: 위치는 유지, 누락 구간은 적분하지 않음
            return
        dt = t - self.prev_t
        dL = (pos[0] - self.prev[0]) * self.r
        dR = (pos[1] - self.prev[1]) * self.r
        self.prev, self.prev_t = pos, t
        ds = (dL + dR) / 2.0
        if self.yaw_source == 'wheel_diff':
            dyaw = (dR - dL) / self.track
        else:
            dyaw = ds * math.tan(self.steer) / self.L
        mid = self.yaw + dyaw / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))

        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        od = Odometry()
        od.header.stamp = js.header.stamp
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = ds / dt
        od.twist.twist.angular.z = dyaw / dt
        pc = [0.0] * 36
        pc[0] = pc[7] = 0.01
        pc[14] = pc[21] = pc[28] = 1e6        # 2D: z, roll, pitch 미사용
        pc[35] = 0.05
        od.pose.covariance = pc
        tc = [0.0] * 36
        tc[0], tc[7], tc[35] = 0.01, 1e6, 0.05
        tc[14] = tc[21] = tc[28] = 1e6
        od.twist.covariance = tc
        self.pub.publish(od)
        if self.publish_tf:
            tf = TransformStamped()
            tf.header = od.header
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tfb.sendTransform(tf)


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
