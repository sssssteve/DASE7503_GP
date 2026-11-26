"""Filter scans and align to the fixed field without commanding the chassis."""
import math
import os
from pathlib import Path
import sys
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from start_region import StartRegion, combined_walls, filter_scan, scan_points
from geometry import wall_match


def pose(position, q):
    return (position.x, position.y,
            math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))


def changed(a, b, distance, angle):
    return math.dist(a[:2], b[:2]) >= distance or abs(math.remainder(a[2]-b[2], math.tau)) >= angle


class FieldLocalization(Node):
    def __init__(self):
        super().__init__('official_field_alignment')
        path = Path(__file__).resolve().parents[2]/'pi_ws/src/dase7503_slam/config/interfaces.yaml'
        mount = yaml.safe_load(path.read_text())['lidar_mount']
        self.mount = [mount[k] for k in ('x','y','z','roll','pitch','yaw')]
        self.grid = self.matcher = self.odom = self.anchor = self.scan = self.sent = None
        config=os.environ.get('DASE_MATCH_CONFIG')
        settings=yaml.safe_load(Path(config).read_text() if config else (Path(__file__).resolve().parents[1]/'match.yaml').read_text())
        self.body_size=settings['base_size_m']
        self.center_x=settings.get('body_center_x_m',0.)
        self.mask_box=settings.get('scan_mask_box_m')
        self.near_gap=settings.get('scan_neighbor_gap_m')
        self.body_padding=settings.get('scan_body_padding_m',0.)
        self.echo_windows=settings.get('scan_echo_windows',[])
        self.bounds=settings.get('start_region') or [2.42,.60,3.02,1.40]
        if os.environ.get('DASE_GUI_START_SIDE')=='1':
            self.bounds=[self.bounds[0],0.,self.bounds[2],2.]
        self.start_heading=settings.get('start_heading_rad')
        self.check = True
        self.stationary_since = self.last_attempt = 0.
        self.candidates = []
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.report = self.create_publisher(String, '/official_alignment/status', latched)
        self.filtered = self.create_publisher(LaserScan, '/scan_filtered', qos_profile_sensor_data)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_service(Trigger, '/official_alignment/check', self.request_check)
        self.create_timer(.10, self.align)

    def on_map(self, msg):
        info = msg.info
        origin = info.origin.position
        if ((info.width, info.height) == (155, 104) and abs(info.resolution-.02) < 1e-5
                and abs(origin.x+.04) < 1e-5 and abs(origin.y+.04) < 1e-5
                and tuple(msg.data[j*155+i] for i,j in ((77,1),(1,52),(153,52),(77,102),(77,52)))
                == (100,100,100,100,0)):
            self.grid = msg

    def on_scan(self, msg):
        filter_scan(msg,self.mount,self.body_size,self.mask_box,self.near_gap,self.echo_windows,self.body_padding,self.center_x)
        if self.anchor is None:
            try:
                self.start_points=scan_points(msg,self.mount)
            except ValueError:
                self.start_points=None
        self.scan = msg
        self.filtered.publish(msg)

    def on_odom(self, msg):
        self.odom = msg
        v = msg.twist.twist
        if math.hypot(v.linear.x, v.linear.y) >= .015 or abs(v.angular.z) >= .025:
            self.stationary_since = 0.
        elif not self.stationary_since:
            self.stationary_since = time.monotonic()

    def status(self, text):
        self.report.publish(String(data=text))

    def request_check(self, _, response):
        self.check = True
        self.candidates.clear()
        response.success = True
        response.message = '已请求复核，停车并收到新扫描后执行'
        return response

    def fresh(self, msg):
        return msg is not None and 0 <= (self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds < 500_000_000

    def align(self):
        now = time.monotonic()
        if any(name == 'slam_toolbox' for name in self.get_node_names()):
            self.candidates.clear()
            self.status('请退出在线 SLAM，再使用官方图定位')
            return
        if not self.fresh(self.odom) or not self.fresh(self.scan):
            self.candidates.clear()
            self.status('等待雷达和底盘里程计')
            return
        odom = pose(self.odom.pose.pose.position, self.odom.pose.pose.orientation)
        if self.anchor is not None and changed(odom, self.anchor, .04, math.radians(4)):
            self.check = True
        if not self.check:
            return
        if not self.stationary_since or now-self.stationary_since < .3:
            self.candidates.clear()
            self.status('移动中由 AMCL 跟踪，停车后复核墙面')
            return
        if now-self.last_attempt < .28 or self.scan.header.stamp == self.sent:
            return
        if self.grid is None or not self.initial.get_subscription_count():
            self.status('等待官方围墙地图和 AMCL')
            return
        msg = self.scan
        stamp = Time.from_msg(msg.header.stamp)
        if msg.header.frame_id != 'laser_frame':
            self.status('雷达坐标系应为 laser_frame')
            return
        try:
            self.tf.lookup_transform('odom', 'base_link', stamp)
        except TransformException:
            return
        self.sent = msg.header.stamp
        self.last_attempt = now
        try:
            u, v = scan_points(msg, self.mount)
            startup = self.anchor is None
            if startup:
                if self.matcher is None:
                    self.matcher = StartRegion(self.grid, self.bounds)
                headings=None if self.start_heading is None else [
                    self.start_heading+math.radians(a) for a in (-8,-4,0,4,8)]
                if self.start_points is None:
                    raise ValueError('车身外回波不足')
                try:
                    prior = self.matcher.estimate(self.start_points, headings)
                except ValueError:
                    origin=(0.,0.,0.)
                    walls=combined_walls([(self.start_points,origin)],origin)
                    prior=self.matcher.estimate(walls,headings)
            else:
                t = self.tf.lookup_transform('map', 'base_link', stamp).transform
                prior = pose(t.translation, t.rotation)
            match = wall_match(list(zip(u,v)), prior)
            x0,y0,x1,y1=self.bounds
            if match is None or (startup and not (x0<=match[0][0]<=x1 and y0<=match[0][1]<=y1)):
                raise ValueError('围墙匹配不在起始区')
            if (startup and self.start_heading is not None and
                    abs(math.remainder(match[0][2]-self.start_heading,math.tau))>math.radians(10)):
                raise ValueError('围墙证据不足，保留当前位姿')
        except (ValueError, TransformException) as error:
            self.candidates.clear()
            self.status(str(error) if isinstance(error, ValueError) else '等待扫描时刻的地图坐标')
            return
        estimate, walls, front = match
        if self.candidates and (changed(estimate, self.candidates[0][0], .05, math.radians(5))
                                or changed(odom, self.candidates[0][1], .015, .025)):
            self.candidates.clear()
        self.candidates.append((estimate, odom))
        if len(self.candidates) < 3:
            return
        self.candidates.clear()
        text = '墙面复核通过 · '+ ' / '.join(walls) + ' · 正前方 '+(front or '未确认')
        if startup or changed(estimate, prior, .025, math.radians(2)):
            x, y, angle = estimate
            out = PoseWithCovarianceStamped()
            out.header.frame_id = 'map'
            out.header.stamp = msg.header.stamp
            p = out.pose.pose
            p.position.x, p.position.y = x, y
            p.orientation.z, p.orientation.w = math.sin(angle/2), math.cos(angle/2)
            out.pose.covariance[0] = out.pose.covariance[7] = .05**2
            out.pose.covariance[35] = math.radians(7)**2
            self.initial.publish(out)
            text+=' · 位姿已提交'
        self.anchor, self.check = odom, False
        self.status(text)


def main():
    rclpy.init()
    node = FieldLocalization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
