"""Browser commands through the existing expiring chassis gate."""
import json
import math
import os
from pathlib import Path
from queue import SimpleQueue
import secrets
import subprocess
import time

from chassis_interfaces.msg import ChassisStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from esp32 import ChassisLink

MANUAL = {'forward': (.08, 0.), 'back': (-.08, 0.),
          'left': (0., .08), 'right': (0., -.08)}


class Drive:
    def __init__(self, node):
        self.node = node
        self.chassis_link = ChassisLink()
        self.chassis_state = {}
        self.queue = SimpleQueue()
        self.feedback_at = self.until = self.manual_until = 0.
        self.ready = False
        self.mode = 'stop'
        self.vector = (0., 0.)
        self.goal = None
        self.request = None
        self.status = '底盘未启用'
        self.owner = node.create_publisher(String, '/control/mode', 10)
        self.lease = node.create_publisher(String, '/control/lease', 10)
        self.manual = node.create_publisher(Twist, '/mapping/cmd_vel', 10)
        self.nav = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.check = node.create_client(Trigger, '/official_alignment/check')
        node.create_subscription(ChassisStatus, '/chassis/status', self.feedback, 10)
        self.config=os.environ.get('DASE_MATCH_CONFIG')
        self.root=Path(__file__).resolve().parents[2]
        self.process=self.race_id=None
        self.race={}
        self.spent=False
        self.map_seeded=False
        self.field=None
        self.next_race_heartbeat=0.
        sticky=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.race_control=node.create_publisher(String,'/match/gui_control',10)
        self.confirmation=node.create_publisher(String,'/match/lift_confirmation',10)
        self.keepout=node.create_publisher(OccupancyGrid,'/match/keepout_map',sticky) if self.config else None
        if self.keepout is not None:
            self.local_map=node.create_publisher(OccupancyGrid,'/match/local_structure_map',sticky)
            node.create_subscription(OccupancyGrid,'/match/gui_local_structure',self.relay_local,sticky)
            node.create_subscription(OccupancyGrid,'/match/gui_keepout',self.relay_keepout,sticky)
        node.create_subscription(String,'/match/gui_status',self.on_race,sticky)
        node.create_timer(.05, self.tick)

    def seed_map(self, msg):
        if self.keepout is None or self.map_seeded:
            return
        import yaml
        from config import qr_post_zones
        from course_clearance import local_structure_map,paint_keepout
        with open(self.config) as stream:
            c=yaml.safe_load(stream)
        standing=[rack['rack_hint'] for rack in c['racks']]
        zones=qr_post_zones(c)
        self.local_map.publish(local_structure_map(msg,zones,standing,c['rack_size_m']))
        paint_keepout(msg,zones,standing,c['rack_size_m'],.04)
        self.keepout.publish(msg)
        self.map_seeded=True
        layout=[dict(name='P'+str(i),center=r['rack_hint'][:2],size=c['rack_size_m'],yaw=r['rack_hint'][2])
                for i,r in enumerate(c['racks'],1)]
        self.field=dict(rack_layout=layout,posts=c['qr_posts'],post_zones=zones,
                        rack_size=c['rack_size_m'],order=[r['code'] for r in c['racks']])
        if c.get('centerline') is not None:
            self.field['centerline']=c['centerline']

    def relay_local(self,msg):
        if self.process is not None and self.map_seeded:self.local_map.publish(msg)

    def relay_keepout(self, msg):
        if self.process is not None and self.map_seeded:
            self.keepout.publish(msg)

    def payload(self):
        return dict(chassis=self.chassis_state, mission=self.race,
                    nav_available=self.nav.server_is_ready(), match_configured=bool(self.config),
                    gate_ready=self.gate_ready(), field_used=self.spent, field=self.field)

    def on_race(self, msg):
        try:
            status=json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(status,dict):
            return
        if status.get('id')==self.race_id:
            self.race=status
            self.spent=self.spent or bool(status.get('started'))

    def race_cmd(self, value):
        if self.race_id:
            self.race_control.publish(String(data=value+':'+self.race_id))

    def start_race(self, now):
        if not self.config or not self.map_seeded or not self.nav.server_is_ready():
            self.status='比赛导航未就绪，请用实车配置启动网页后端'
            return
        if self.spent or self.process is not None or 'real_rack_match' in self.node.get_node_names():
            self.status='已有比赛流程，或场地需要清场并重启'
            return
        if (self.mode!='stop' or not self.ready or now-self.feedback_at>.35 or
                not self.gate_ready() or self.node.snapshot()['pose'] is None):
            self.status='开始比赛前需停止遥控，并恢复底盘反馈与地图定位'
            return
        self.race_id=secrets.token_hex(12)
        env=dict(os.environ,DASE_GUI_MATCH_ID=self.race_id)
        try:
            with (Path(__file__).resolve().parent/'gui-match.log').open('ab') as stream:
                self.process=subprocess.Popen(['bash',str(self.root/'pi_hardware/pi.sh'),'run',self.config],
                    env=env,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        except OSError as error:
            self.status='无法启动比赛流程：'+str(error)
            self.race_id=None
            return
        self.race={'state':'checking'}
        self.mode='race'
        self.until=now+2.
        self.next_race_heartbeat=now
        self.status='比赛流程已启动，等待相机与导航检查'

    def race_tick(self, now):
        result=self.process.poll()
        if result is not None:
            if self.race.get('state') not in ('done','failed'):
                self.spent=True
                self.race={'state':'failed','reason':'比赛进程异常退出，请查看 gui-match.log'}
            self.process=None
            self.mode='stop'
            self.status='比赛结束' if result==0 else self.race.get('reason','比赛已停止')
            return
        if self.mode=='stopping' or now>=self.until:
            if self.mode!='stopping':
                self.stop('网页控制连接超时，正在停车')
            self.race_cmd('stop')
            return
        if now>=self.next_race_heartbeat:
            self.race_cmd('live')
            self.next_race_heartbeat=now+.25

    def feedback(self, msg):
        self.feedback_at = time.monotonic()
        self.ready = not (msg.fault or msg.emergency_stopped)

    def gate_ready(self):
        pubs = self.node.get_publishers_info_by_topic('/cmd_vel')
        return (len(pubs) == 1 and pubs[0].node_name == 'chassis_odom_and_control_gate'
                and self.owner.get_subscription_count() == 1)

    def stop(self, reason='已停止'):
        if self.process is not None:
            self.owner.publish(String(data='stop'))
            self.race_cmd('stop')
            self.mode='stopping'
            self.status=reason
            return
        if self.mode != 'stop':
            self.owner.publish(String(data='stop'))
        if self.goal is not None:
            self.goal.cancel_goal_async()
        self.mode, self.goal, self.request = 'stop', None, None
        self.vector = (0., 0.)
        self.status = reason

    def command(self, op, data, now):
        if op == 'stop':
            self.stop()
        elif op == 'chassis':
            if self.process is None and not (self.ready and now-self.feedback_at<.35):
                self.chassis_link.start(now)
            self.status=self.chassis_link.report(now,self.feedback_at,self.ready)['text']
        elif op == 'race':
            self.start_race(now)
        elif op == 'confirm':
            if self.mode=='race' and self.race.get('state')=='confirm' and self.race.get('token')==data.get('token'):
                self.confirmation.publish(String(data=data['token']))
        elif op == 'check':
            if self.check.service_is_ready():
                self.check.call_async(Trigger.Request())
            else:
                self.status = '墙面复核服务未就绪'
        elif op in ('manual', 'nav'):
            if self.process is not None or self.spent or 'real_rack_match' in self.node.get_node_names():
                self.status='比赛占用底盘，请先停车或清场重启'
                return
            self.stop()
            if not self.ready or now-self.feedback_at > .35 or not self.gate_ready():
                self.status = '等待正常的 ESP32 反馈和唯一的底盘控制节点'
            elif op == 'nav' and not self.nav.server_is_ready():
                self.status = 'Nav2 未启动'
            else:
                self.mode, self.until = op, now+.7
                if op == 'manual':
                    self.lease.publish(String(data='gui'))
                    self.owner.publish(String(data='mapping'))
                self.status = '遥控已启用' if op == 'manual' else '导航已启用，请在地图选择目标'
        elif op == 'heartbeat':
            self.until = now+(1.4 if self.mode=='race' else .7)
        elif op == 'release':
            self.vector = (0., 0.)
        elif op in MANUAL and self.mode == 'manual':
            self.vector, self.manual_until = MANUAL[op], now+.22
        elif op == 'goal' and self.mode == 'nav' and self.request is None and self.goal is None:
            x,y = data['x'],data['y']
            if self.node.snapshot()['pose'] is None or not (.12 < x < 2.90 and .12 < y < 1.88):
                self.status = '目标点或实时定位不可用'
                return
            yaw = self.node.snapshot()['pose'][2]
            request = NavigateToPose.Goal()
            request.pose.header.frame_id = 'map'
            request.pose.header.stamp = self.node.get_clock().now().to_msg()
            p = request.pose.pose
            p.position.x, p.position.y = x,y
            p.orientation.z, p.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            self.request = self.nav.send_goal_async(request)
            self.request.add_done_callback(self.accept)
            self.status = '等待 Nav2 确认目标'

    def accept(self, future):
        if future.exception() is not None:
            if future is self.request:
                self.request = None
                self.status = 'Nav2 目标请求中断'
            return
        handle = future.result()
        if future is not self.request:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        self.request = None
        if not handle.accepted:
            self.status = 'Nav2 拒绝目标'
            return
        self.goal = handle
        now = time.monotonic()
        if now > self.until or not self.ready or now-self.feedback_at > .35 or not self.gate_ready():
            self.stop('底盘反馈或控制连接中断，已取消目标')
            return
        self.lease.publish(String(data='gui'))
        self.owner.publish(String(data='nav'))
        self.status = '导航进行中'
        handle.get_result_async().add_done_callback(lambda result: self.finish(handle, result))

    def finish(self, handle, result):
        if handle is self.goal:
            self.goal = None
            self.owner.publish(String(data='stop'))
            status = None if result.exception() is not None else result.result().status
            self.status = {4:'到达目标', 5:'目标已取消'}.get(status, '导航失败')

    def tick(self):
        now = time.monotonic()
        self.chassis_state=self.chassis_link.report(now,self.feedback_at,self.ready)
        while not self.queue.empty():
            op, data, received = self.queue.get_nowait()
            if op in ('release','stop') or now-received < .4:
                self.command(op, data, now)
        if self.process is not None:
            self.race_tick(now)
            return
        if self.mode == 'stop':
            return
        if (now > self.until or not self.ready or now-self.feedback_at > .35 or
                not self.gate_ready() or (self.mode == 'nav' and self.node.snapshot()['pose'] is None)):
            self.stop('连接、底盘反馈或定位中断，已停车')
            return
        if self.mode == 'nav' and self.goal is None:
            return
        self.lease.publish(String(data='gui'))
        if self.mode == 'manual':
            msg = Twist()
            if now <= self.manual_until:
                msg.linear.x, msg.linear.y = self.vector
            self.manual.publish(msg)
