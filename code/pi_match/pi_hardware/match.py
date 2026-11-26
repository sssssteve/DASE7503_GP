"""Real contest coordinator and read-only live readiness check (ROS 2 Jazzy)."""
import argparse
import json
import math
import os
import secrets
import select
import signal
import sys
import termios
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from geometry_msgs.msg import Point32,Polygon,PolygonStamped,PoseStamped,PoseWithCovarianceStamped,Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import BackUp, ComputePathToPose, FollowPath, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid,Odometry,Path
from sensor_msgs.msg import Image,LaserScan
from std_msgs.msg import Bool,String,UInt32
from tf2_ros import Buffer,TransformException,TransformListener
from chassis_interfaces.msg import ChassisStatus,RelativeMove,ServoCommand,ServoStatus
from drop_region import candidates, obstacle_grid, verify_region_pose
from centerline import check_detour_scan, start_passage
from config import load
from course_clearance import (
    MARGIN, NAV_GOAL_TOLERANCE_M, box_distance, check_corridor, grid_cell, in_keepout, keepout_cells, overlap, local_structure_map, paint_keepout, rack_pose, relative, world,
    IncompleteCoverage, rear_strip_blockage, verify_drop, verify_retreat, verify_travel,
)
from rack_survey import fit_rack, stable_legs, insertion_error, measure_spacing, observed_opening
from official_localization.geometry import leg_centers

AISLE_HEADING_TOLERANCE=math.radians(25)

class RetreatBlocked(RuntimeError):
    pass


class RackViewLost(RuntimeError):
    pass


class Match(Node):
    def __init__(self,c,nav_only=False,camera_prechecked=False):
        super().__init__('real_rack_match')
        self.nav_only=nav_only
        self.direct_test=nav_only and os.environ.get('DASE_DIRECT_NAV_TEST')=='1'
        self.c=c
        self.last={}
        self.receipt={}
        self.deadline=None
        self.running=False
        self.wait_links=False
        self.paused_seconds=0.
        self.reconnections=0
        self.last_map_pose=None
        self.last_good_odom=None
        self.start_odom_offset=(0.,0.,0.)
        self.lift_target=None
        self.recovered_lift=False
        self.nav_goal=None
        self.relative_id=None
        self.last_lift_angle=None
        self.servo_id=0
        self.deposited=[]
        self.standing={rack['code']:rack['rack_hint'] for rack in c['racks']}
        self.carrying=False
        self.payload_legs=[]
        self.last_travel_scan=None
        self.relative_path=None
        self.last_guard_scan=None
        self.tf=Buffer()
        self.listener=TransformListener(self.tf,self)
        self.fixed=c.get('drop_policy')=='fixed'
        self.strafe_peaks=None
        self.nav=(ActionClient(self,FollowPath,'/follow_path') if self.fixed else
                  ActionClient(self,NavigateToPose,'/navigate_to_pose'))
        self.life=self.create_client(GetState,'/bt_navigator/get_state')
        sticky=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sources=[
            ('chassis',ChassisStatus,'/chassis/status',1),('odom',Odometry,'/odom',1),
            ('map',OccupancyGrid,'/map',sticky),
            ('costmap',OccupancyGrid,'/global_costmap/costmap',qos_profile_sensor_data),('scan',LaserScan,'/scan_body_filtered',qos_profile_sensor_data),
            ('image',Image,'/camera/image_raw',qos_profile_sensor_data),
            ('gate',String,'/control/status',10),('vision',String,'/vision/status',10),
            ('servo',ServoStatus,'/servo/status',10),
            ('operator',String,'/match/lift_confirmation',10),
            ('local_shape',PolygonStamped,'/local_costmap/published_footprint',qos_profile_sensor_data),
            ('global_shape',PolygonStamped,'/global_costmap/published_footprint',qos_profile_sensor_data)]
        self.check_only=[]
        for key,kind,topic,qos in sources:
            if key=='image' and camera_prechecked:continue
            subscription=self.create_subscription(kind,topic,lambda msg,k=key:self.record(k,msg),qos)
            if key=='image':
                self.check_only.append(subscription)
        self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose') if self.fixed or any(
            r['slot'] is None for r in c['racks']) else None
        self.map_goals=[c['end_approach']]+[r['approach'] for r in c['racks']]
        self.map_goals.extend(r['slot'] for r in c['racks'] if r['slot'] is not None)
        self.map_goals.extend(view for rack in c['racks'] for view in rack['scan_views'])
        self.centerline=c.get('centerline')
        self.backup_client=ActionClient(self,BackUp,'/backup')
        self.turn=ActionClient(self,Spin,'/spin')
        if self.centerline:
            aisle=self.centerline
            self.map_goals.extend([(aisle.entry_x,aisle.y,0.),(aisle.end_x,aisle.y,0.)])
            self.map_goals.extend(aisle.rack_gate(r['approach']) for r in c['racks'])
        if c.get('start_region') is None:
            self.map_goals.append(c['initial_pose'])
        else:
            self.initial_pose_pub=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
            self.amcl_life=self.create_client(GetState,'/amcl/get_state')
            self.localization_status=self.create_publisher(String,'/match/localization_status',sticky)
        self.route_pub=self.create_publisher(String,'/match/route_preview',sticky)
        self.route_preview={}
        self.commands={}
        self.keepout_topic='/match/gui_keepout' if os.environ.get('DASE_GUI_MATCH_ID') else '/match/keepout_map'
        self.local_structure_topic='/match/gui_local_structure' if os.environ.get('DASE_GUI_MATCH_ID') else '/match/local_structure_map'
        for topic,kind,qos in [
            ('/control/lease',String,1),('/control/mode',String,1),('/match/scan_target',String,10),
            (self.keepout_topic,OccupancyGrid,sticky),(self.local_structure_topic,OccupancyGrid,sticky),('/lidar/cmd_vel',Twist,1),('/chassis/move',RelativeMove,10),
            ('/chassis/cancel',UInt32,10),('/chassis/estop',Bool,10),('/servo/command',ServoCommand,10),
            ('/local_costmap/footprint',Polygon,sticky),('/global_costmap/footprint',Polygon,sticky)]:
            if nav_only and topic=='/servo/command':
                continue
            self.commands[topic]=self.create_publisher(kind,topic,qos)
        self.lease='stop'
        self.last_renew=0.
        self.gui_id=os.environ.get('DASE_GUI_MATCH_ID')
        self.phase_state={}
        self.completed=[]
        self.target_legs=[]
        self.current_rack=None
        self.gui_until=time.monotonic()+5
        self.gui_stopped=False
        self.next_status=0.
        if self.gui_id:
            self.progress=self.create_publisher(String,'/match/gui_status',sticky)
            self.create_subscription(String,'/match/gui_control',self.gui_control,10)

    def report_route(self,status,*,action=None,goal=None,points=()):
        if action is not None:
            self.route_preview=dict(action=action,goal=goal,points=points)
        self.route_preview['status']=status
        self.route_pub.publish(String(data=json.dumps(self.route_preview,ensure_ascii=False)))

    def preview_follow(self,label,pose,req):
        points=[(p.pose.position.x,p.pose.position.y) for p in req.path.poses] if self.fixed else []
        self.report_route('已提交',action=label,goal=pose,points=points)

    def relative_preview(self,label,meters=0.,turn=0.):
        base=self.last_map_pose
        if base is None:
            self.report_route('准备',action=label)
            return
        goal=(*world((meters,0.),base),math.remainder(base[2]+turn,math.tau))
        self.report_route('准备',action=label,goal=goal,points=[base[:2],goal[:2]])

    def gui_control(self,msg):
        if msg.data=='live:'+self.gui_id:
            self.gui_until=time.monotonic()+2
        elif msg.data=='stop:'+self.gui_id:
            self.gui_stopped=True

    def publish_progress(self):
        self.next_status=time.monotonic()+.5
        self.progress.publish(String(data=json.dumps(dict(
            id=self.gui_id, rack=self.current_rack, order=[r['code'] for r in self.c['racks']],
            completed=self.completed, placed=self.deposited, target_legs=self.target_legs,
            carrying=self.carrying, started=self.deadline is not None,
            remaining=None if self.deadline is None else max(0,math.ceil(self.deadline-time.monotonic())),
            **self.phase_state),ensure_ascii=False)))

    def phase(self,state,**details):
        if self.gui_id:
            self.phase_state={'state':state, **details}
            self.publish_progress()

    def record(self,key,msg):
        self.receipt[key]=time.monotonic()
        if key=='chassis':self.record_strafe_vy(key,msg.vy_mps)
        if key in ('gate','vision'):
            try:msg=json.loads(msg.data)
            except (TypeError,ValueError):msg={}
        self.last[key]=msg

    def record_strafe_vy(self,key,value):
        if self.strafe_peaks is not None and math.isfinite(value):
            if key not in self.strafe_peaks or abs(value)>abs(self.strafe_peaks[key]):
                self.strafe_peaks[key]=value

    def send(self,topic,msg):
        self.commands[topic].publish(msg)

    def fresh(self,key,limit):
        return key in self.last and time.monotonic()-self.receipt[key]<limit

    def device_gap(self):
        for key,limit in [('chassis',.5),('odom',.5),('scan',.6)]:
            if not self.fresh(key,limit):
                return key+' 数据过期'
        for key in ('odom','scan'):
            age=(self.get_clock().now()-Time.from_msg(self.last[key].header.stamp)).nanoseconds/1e9
            if age>.6:
                return key+' 采样过期'
        return None

    def sensor_fault(self,*,include_gap=True):
        if include_gap:
            gap=self.device_gap()
            if gap:return gap
        if not self.fresh('gate',.5):return '底盘仲裁数据过期'
        cs=self.last['chassis']
        if cs.fault or cs.emergency_stopped:return 'ESP32 故障或急停'
        if self.last['gate'].get('cmd_publishers')!=1:return '/cmd_vel 发布者数量异常'
        for k in ('odom','scan'):
            if (self.get_clock().now()-Time.from_msg(self.last[k].header.stamp)).nanoseconds < -50_000_000:
                return k+' 时间戳在未来'
        if self.last['scan'].header.frame_id!='laser_frame':return '/scan 坐标系不匹配'
        scan=self.last['scan']
        if sum(math.isfinite(r) and scan.range_min<=r<=scan.range_max for r in scan.ranges)<10:return '有效雷达回波不足'
        if self.last['odom'].header.frame_id!='odom' or self.last['odom'].child_frame_id!='base_link':return '/odom 坐标系不匹配'
        return None

    def poll(self):
        rclpy.spin_once(self,timeout_sec=.03)
        if self.gui_id:
            now=time.monotonic()
            if self.gui_stopped or now>self.gui_until:
                raise RuntimeError('网页已停止或连接超时')
            if now>=self.next_status:
                self.publish_progress()
        if self.deadline and time.monotonic()>=self.deadline:
            raise TimeoutError('比赛计时结束')

    def work_time(self):
        return time.monotonic()-self.paused_seconds

    def startup_odom(self,x,y,yaw):
        dx,dy,a=self.start_odom_offset
        return dx+math.cos(a)*x-math.sin(a)*y,dy+math.sin(a)*x+math.cos(a)*y,math.remainder(a+yaw,math.tau)

    def wait_for_devices(self,reason=None):
        if not self.wait_links:
            return
        reason=reason or self.device_gap()
        if not reason and self.last_good_odom:
            cs=self.last['chassis']
            x,y,a=self.last_good_odom
            if math.hypot(cs.x_m-x,cs.y_m-y)>.20 or abs(math.remainder(cs.yaw_rad-a,math.tau))>.3:
                reason='底盘里程计跳变'
        if not reason:
            return
        began=time.monotonic()
        print('设备暂停：'+reason+'，停车等待，保留当前步骤',flush=True)
        old_phase=self.phase_state.copy()
        self.report_route('等待设备',action='停车')
        self.phase('waiting_devices',reason=reason)
        self.lease='stop'
        if self.nav_goal and self.nav_goal.accepted:
            self.nav_goal.cancel_goal_async()
        stamp=None
        consecutive=0
        next_stop=0.
        reset=False
        chassis_gap=(reason=='底盘里程计跳变' or not self.fresh('chassis',.5))
        seed_at=0.
        load_uncertain=False
        token=None
        while consecutive<3:
            now=time.monotonic()
            if now>=next_stop:
                self.send('/control/mode',String(data='stop'))
                self.send('/control/lease',String(data='stop'))
                if self.relative_id:
                    self.send('/chassis/cancel',UInt32(data=self.relative_id))
                next_stop=now+.12
            self.poll()
            chassis_gap |= not self.fresh('chassis',.5)
            if (self.carrying or self.lift_target is not None) and chassis_gap:
                if not load_uncertain:
                    print('载货或升降时底盘反馈中断，当前硬件无法确认承载状态，保持停车',flush=True)
                load_uncertain=True
            if self.device_gap() or not self.fresh('gate',.5):
                consecutive=0
                continue
            cs=self.last['chassis']
            if self.sensor_fault() or self.last['gate'].get('mode')!='stop' or cs.action_state==ChassisStatus.ACTION_RUNNING:
                consecutive=0
                continue
            scan=self.last['scan']
            if scan.header.stamp==stamp:continue
            stamp=scan.header.stamp
            if max(abs(cs.vx_mps),abs(cs.vy_mps),abs(cs.wz_rad_s))>=.015:
                consecutive=0
                continue
            if self.running:
                stamp_time=Time.from_msg(stamp)
                if (not self.tf.can_transform('map','base_link',stamp_time) or
                        not self.tf.can_transform('map',scan.header.frame_id,stamp_time)):
                    consecutive=0
                    continue
                if self.last_good_odom:
                    x,y,a=self.last_good_odom
                    reset |= chassis_gap and (math.hypot(cs.x_m-x,cs.y_m-y)>.015 or
                                              abs(math.remainder(cs.yaw_rad-a,math.tau))>.035)
                if self.last_map_pose and reset:
                    # An ESP32 reboot can zero its encoders. Verify a map-frame
                    # pose from walls before allowing any old goal to resume.
                    from official_localization.geometry import wall_match
                    from start_region import scan_points
                    try:
                        points=scan_points(scan,self.c['laser_mount'])
                    except ValueError:
                        consecutive=0
                        continue
                    fit=wall_match(list(zip(*points)),self.last_map_pose)
                    if fit is None:
                        consecutive=0
                        continue
                    expected=fit[0]
                    try:
                        t=self.tf.lookup_transform('map','base_link',stamp_time).transform
                    except TransformException:
                        consecutive=0
                        continue
                    q=t.rotation
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    if (math.hypot(t.translation.x-expected[0],t.translation.y-expected[1])>.035 or
                            abs(math.remainder(yaw-expected[2],math.tau))>.055):
                        consecutive=0
                        if now>=seed_at and hasattr(self,'initial_pose_pub'):
                            self.publish_initial_pose(*expected,stamp=scan.header.stamp)
                            seed_at=now+2.
                        continue
            consecutive+=1
            if load_uncertain and consecutive>=3:
                up=self.carrying if self.lift_target is None else self.lift_target
                if token is None:
                    token=self.request_load_confirmation(up)
                if not self.load_confirmed(token):
                    consecutive=2
                    continue
                if self.lift_target is not None:self.recovered_lift=True
        if not self.running and self.last_good_odom:
            x,y,a=self.last_good_odom
            if math.hypot(cs.x_m-x,cs.y_m-y)>.08 or abs(math.remainder(cs.yaw_rad-a,math.tau))>.12:
                x,y,a=self.startup_odom(x,y,a)
                offset=math.remainder(a-cs.yaw_rad,math.tau)
                self.start_odom_offset=(x-math.cos(offset)*cs.x_m+math.sin(offset)*cs.y_m,
                                        y-math.sin(offset)*cs.x_m-math.cos(offset)*cs.y_m,offset)
        self.last_good_odom=(cs.x_m,cs.y_m,cs.yaw_rad)
        self.paused_seconds+=time.monotonic()-began
        self.reconnections+=1
        self.phase_state=old_phase
        self.last_travel_scan=self.last_guard_scan=None
        print('设备已恢复并停稳，继续当前步骤',flush=True)

    def spin(self):
        self.poll()
        self.wait_for_devices()
        if self.fresh('chassis',.5):
            cs=self.last['chassis']
            self.last_good_odom=(cs.x_m,cs.y_m,cs.yaw_rad)
        if self.running:
            gap=self.device_gap()
            if gap and self.wait_links:
                self.wait_for_devices(gap)
                return
            fault=gap or self.sensor_fault(include_gap=False)
            if fault:raise RuntimeError(fault)
            if self.lease=='lidar' and self.last_travel_scan!=self.last['scan'].header.stamp:
                self.check_alignment_travel()
            if self.lease=='relative' and self.relative_path is not None and self.last_guard_scan != self.last['scan'].header.stamp:
                self.check_relative()
            if time.monotonic()-self.last_renew>.12:
                self.send('/control/lease',String(data=self.lease))
                self.last_renew=time.monotonic()

    def wait(self,fn,seconds,reason):
        until=None if seconds is None else self.work_time()+seconds
        while not fn():
            if until is not None and self.work_time()>=until:raise TimeoutError(reason)
            self.spin()

    def pause(self,seconds):
        until=self.work_time()+seconds
        self.wait(lambda:self.work_time()>=until,seconds+.5,'等待超时')

    def future(self,f,seconds,reason):
        self.wait(f.done,seconds,reason)
        return f.result()

    def wait_active(self,client,name):
        while True:
            pending=client.call_async(GetState.Request())
            try:
                state=self.future(pending,5,name+' 状态查询超时')
            except TimeoutError:
                if not self.wait_links:raise
                pending.cancel()
            else:
                if state.current_state.label=='active':return
                if not self.wait_links:raise RuntimeError(name+' 尚未 active')
            if hasattr(self,'localization_status'):
                self.localization_status.publish(String(data='等待 '+name+' 激活'))
            self.pause(.5)

    def localize_start(self, allow_turn=False):
        from start_region import StartRegion, combined_walls, scan_points

        def angle(q):
            return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        def odom_pose():
            p=self.last['odom'].pose.pose
            return self.startup_odom(p.position.x,p.position.y,angle(p.orientation))

        def difference(a,b):
            return math.dist(a[:2],b[:2]),abs(math.remainder(a[2]-b[2],math.tau))

        def stationary():
            if self.sensor_fault():
                return False
            cs=self.last['chassis']
            return (self.last['gate'].get('mode')=='stop'
                    and cs.action_state!=ChassisStatus.ACTION_RUNNING
                    and max(abs(cs.vx_mps),abs(cs.vy_mps),abs(cs.wz_rad_s))<.015)

        self.wait(lambda:stationary() and self.last.get('map') is not None
                  and self.last['map'].header.frame_id=='map'
                  and self.initial_pose_pub.get_subscription_count()
                  and self.amcl_life.service_is_ready(),None if self.wait_links else 20,'起始定位需要静止底盘、地图、激光和已启动的 AMCL')
        self.wait_active(self.amcl_life,'AMCL')
        matcher=StartRegion(self.last['map'],self.c['start_region'])
        self.wait(stationary,3,'请保持车静止')
        reference=odom_pose()
        stamp=self.last['scan'].header.stamp
        heading=self.c.get('start_heading_rad')
        headings=None if heading is None else [heading+math.radians(a) for a in (-8,-4,0,4,8)]
        mount=self.c['laser_mount']
        def capture():
            nonlocal stamp
            self.wait(lambda:stationary() and self.last['scan'].header.stamp!=stamp,
                      3,'未收到新鲜的静止扫描')
            scan=self.last['scan']
            stamp=scan.header.stamp
            pose=odom_pose()
            try:
                points=scan_points(scan,mount)
            except ValueError:
                points=([],[])
            return points,pose

        def turn_to(target):
            # The existing velocity gate expires stale commands and leases.
            after_progress=self.work_time()
            best=math.inf
            blocked=False
            try:
                while True:
                    self.wait_for_devices()
                    fault=self.sensor_fault()
                    if fault or self.last['chassis'].action_state==ChassisStatus.ACTION_RUNNING:
                        raise RuntimeError(fault or '底盘有其他动作')
                    pose=odom_pose()
                    error=math.remainder(target-pose[2],math.tau)
                    if abs(error)<.025:
                        break
                    if math.dist(reference[:2],pose[:2])>.05:
                        raise RuntimeError('找墙期间检测到异常平移')
                    points,_=self.scan_points(self.last['scan'],'base_link')
                    near=any(math.hypot(x,y)<self.c['radius_empty_m']+.025 for x,y in points)
                    if near:
                        if not blocked:
                            self.mode('stop')
                            print('起始找墙：旋转范围内有回波，停车等待净空',flush=True)
                        blocked=True
                        self.pause(.08)
                        after_progress=self.work_time()
                        continue
                    if self.lease=='stop':
                        self.mode('mapping')
                    elif self.last['gate'].get('mode')!='mapping':
                        raise RuntimeError('找墙时速度控制权丢失')
                    blocked=False
                    if abs(error)<best-.008:
                        best=abs(error)
                        after_progress=self.work_time()
                    if self.work_time()-after_progress>2:
                        raise RuntimeError('找墙旋转未收到预期的里程计角度变化')
                    command=Twist()
                    command.angular.z=math.copysign(min(.20,max(.075,abs(error))),error)
                    self.send('/control/lease',String(data='mapping'))
                    turn_pub.publish(command)
                    self.pause(.06)
            finally:
                turn_pub.publish(Twist())
                self.stop_after_motion()
            self.wait(stationary,3,'找墙后底盘未停稳')
            self.pause(.22)

        history=[[],[],[]]
        recovered=False
        self.localization_status.publish(String(data='停车采样，匹配起始区围墙'))
        while True:
            estimates=[]
            reference_at_stop=odom_pose()
            reason='连续静止扫描不一致'
            for attempt in range(6):
                points,pose=capture()
                dp,da=difference(reference_at_stop,pose)
                if dp>.015 or da>.02:
                    raise RuntimeError('定位期间底盘意外移动')
                try:
                    cloud=(combined_walls(history[attempt%3]+[(points,pose)],pose)
                           if history[attempt%3] or (recovered and attempt>=3) else points)
                    if len(points[0])<32:
                        raise ValueError('当前有效回波不足')
                    candidate=matcher.estimate(cloud,headings)
                except ValueError as error:
                    estimates.clear()
                    reason=str(error)
                    continue
                if estimates:
                    dp,da=difference(estimates[0],candidate)
                    if dp>.07 or da>math.radians(10):
                        estimates.clear()
                estimates.append(candidate)
                if len(estimates)==3:
                    break
            if len(estimates)==3:
                x,y,yaw=estimates[-1]
                self.publish_initial_pose(x,y,yaw,stamp)
                text=f'已提交激光初始位姿 x={x:.2f} m, y={y:.2f} m, yaw={math.degrees(yaw):.1f}°，进入导航就绪检查'
                self.localization_status.publish(String(data=text))
                print(text,flush=True)
                return
            if not allow_turn:
                raise RuntimeError('起始定位未通过：'+reason)
            if recovered:
                # Keep checking new scans at the original QR heading.
                print('已回到起步朝向，继续等待可靠墙面匹配：'+reason,flush=True)
                history=[[],[],[]]
                self.pause(1)
                continue
            turn_pub=self.create_publisher(Twist,'/mapping/cmd_vel',1)
            self.wait(lambda:turn_pub.get_subscription_count()>0,3,'起始旋转缺少速度仲裁')
            self.pause(.2)
            if self.count_publishers('/mapping/cmd_vel')!=1:
                raise RuntimeError('起始找墙与其他遥控发布者冲突')
            self.localization_status.publish(String(data='低速原地找墙：'+reason))
            print('起始定位证据不足，开始低速原地找墙：'+reason,flush=True)
            try:
                for step in range(1,9):
                    turn_to(reference[2]+step*math.tau/8)
                    print(f'起始找墙 {step}/8，停车采样',flush=True)
                    for bank in history:
                        bank.append(capture())
            finally:
                self.destroy_publisher(turn_pub)
            recovered=True
    def publish_initial_pose(self,x,y,yaw,stamp=None):
        message=PoseWithCovarianceStamped()
        message.header.frame_id='map'
        message.header.stamp=self.get_clock().now().to_msg() if stamp is None else stamp
        message.pose.pose.position.x=x
        message.pose.pose.position.y=y
        message.pose.pose.orientation.z=math.sin(yaw/2)
        message.pose.pose.orientation.w=math.cos(yaw/2)
        message.pose.covariance[0]=message.pose.covariance[7]=.05**2
        message.pose.covariance[35]=math.radians(7)**2
        self.initial_pose_pub.publish(message)
        return message

    def ready(self):
        def problem():
            fault=self.sensor_fault()
            if fault:return fault
            if self.check_only and not self.fresh('image',1.):return '扫码相机画面缺失'
            m=self.last.get('map')
            if m is None or not m.info.width or m.header.frame_id!='map':return '缺少 map'
            q=m.info.origin.orientation
            yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
            for gx,gy,_ in self.map_goals:
                x=gx-m.info.origin.position.x;y=gy-m.info.origin.position.y
                i=math.floor((math.cos(yaw)*x+math.sin(yaw)*y)/m.info.resolution)
                j=math.floor((-math.sin(yaw)*x+math.cos(yaw)*y)/m.info.resolution)
                if not (0<=i<m.info.width and 0<=j<m.info.height and 0<=m.data[j*m.info.width+i]<25):
                    return f'配置目标 ({gx}, {gy}) 不在已知空闲格'
            stamp=Time.from_msg(self.last['scan'].header.stamp)
            if not self.tf.can_transform('map','laser_frame',stamp):return '扫描时刻缺少 map→laser_frame'
            for topic,pub in self.commands.items():
                if not pub.get_subscription_count():return '缺少订阅者 '+topic
            if not self.nav.server_is_ready() or not self.life.service_is_ready():return 'Nav2 尚未就绪'
            if self.planner is not None and not self.planner.server_is_ready():return '卸货选位规划器未就绪'
            return None
        until=self.work_time()+20
        last_issue=None
        while issue:=problem():
            if issue!=last_issue:
                if hasattr(self,'localization_status'):
                    self.localization_status.publish(String(data='导航就绪检查：'+issue))
                last_issue=issue
            if self.wait_links and (issue in ('扫描时刻缺少 map→laser_frame','Nav2 尚未就绪','卸货选位规划器未就绪','缺少 map') or issue.startswith('缺少订阅者 ')):
                until=self.work_time()+20
            if self.work_time()>until:raise RuntimeError(issue)
            self.spin()
        self.wait_active(self.life,'bt_navigator')
        if self.last['chassis'].action_state==ChassisStatus.ACTION_RUNNING:raise RuntimeError('底盘仍在执行其他动作')
        if self.gui_id:
            sample=self.last['scan']
            t=self.tf.lookup_transform('map','base_link',Time.from_msg(sample.header.stamp)).transform
            x0,y0,x1,y1=self.c.get('start_region') or (2.42,.60,3.02,1.40)
            q=t.rotation
            angle=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
            heading=self.c.get('start_heading_rad')
            if (not (x0 <= t.translation.x <= x1 and y0 <= t.translation.y <= y1)
                    or (heading is not None and abs(math.remainder(angle-heading,math.tau))>math.radians(12))):
                raise RuntimeError('请把车放回蓝色 START 区，车头朝约定的扫码方向')
        if self.fixed and not self.nav_only:
            parked=[]
            racks=self.c['racks']
            for i,rack in enumerate(racks):
                remaining=racks[i+1:]
                occupied=parked+[r['rack_hint'] for r in remaining]
                reserved=[(self.c['end_approach'],self.c['radius_empty_m'])]
                reserved.extend((r['slot'],self.c['radius_loaded_m']) for r in remaining)
                if self.centerline:
                    reserved.append(((self.centerline.end_x,self.centerline.y),self.c['radius_loaded_m']))
                verify_region_pose(self.c,rack['slot'],occupied,self.last['map'],reserved)
                parked.append(rack_pose(rack['slot'],self.c['load_center_m']))
        self.publish_keepout()
        if hasattr(self,'localization_status'):
            self.localization_status.publish(String(data='实时定位与导航检查通过'))
        print('实时检查通过。请在 RViz 核对初始定位、激光与地图对齐。',flush=True)

    def mode(self,name):
        self.lease=name
        self.send('/control/lease',String(data=name))
        self.send('/control/mode',String(data=name))
        after=time.monotonic()
        generation=self.reconnections
        self.wait(lambda:generation!=self.reconnections or
                  (self.receipt['gate']>=after and self.last['gate'].get('mode')==name),1,'速度仲裁未切换')
        if name=='stop':
            self.wait(lambda:max(abs(self.last['chassis'].vx_mps),abs(self.last['chassis'].vy_mps),abs(self.last['chassis'].wz_rad_s))<.02,
                      3,'底盘未停稳')

    def stop_after_motion(self):
        if sys.exc_info()[0] is None:
            self.mode('stop')
        else:
            self.lease='stop'
            self.send('/control/mode',String(data='stop'))
            self.send('/control/lease',String(data='stop'))

    def checked_route(self, goal):
        generation=self.reconnections
        self.mode('stop')
        planners=('AnyAngle','GridBased') if self.c.get('path_mode')=='straight' else ('GridBased',)
        blocked_codes=[]
        for planner in planners:
            req=ComputePathToPose.Goal()
            req.goal=goal
            req.planner_id=planner
            handle=self.future(self.planner.send_goal_async(req),4,'规划请求超时')
            if not handle.accepted:
                continue
            reply=self.future(handle.get_result_async(),8,'规划超时')
            if reply.status!=4:
                code=reply.result.error_code
                if code in (ComputePathToPose.Result.START_OCCUPIED,ComputePathToPose.Result.GOAL_OCCUPIED,
                            ComputePathToPose.Result.NO_VALID_PATH):
                    blocked_codes.append(code)
                continue
            path=reply.result.path
            if path.header.frame_id!='map' or not path.poses:
                continue
            if any(p.header.frame_id not in ('','map') for p in path.poses):
                continue
            first=path.poses[0].pose.position
            last=path.poses[-1].pose.position
            end=goal.pose.position
            if (math.dist(self.map_base_pose()[:2],(first.x,first.y))>.10 or
                    math.hypot(end.x-last.x,end.y-last.y)>.06):
                blocked_codes.append(ComputePathToPose.Result.NO_VALID_PATH)
                continue
            # The planner owns the detour/endpoint; the task owns the final camera and tray heading.
            path.poses[-1].pose.orientation=goal.pose.orientation
            following=FollowPath.Goal()
            following.path=path
            following.controller_id='FollowPath'
            return following
        if generation!=self.reconnections:return None
        if blocked_codes:
            print(f'Nav2 暂无可行路径 {blocked_codes}，停车等待重规划',flush=True)
            return None
        raise RuntimeError('Nav2 未返回连接当前位姿与目标的有效路径')

    def strafe_request(self,goal,side):
        # Directional DWB follows a straight body-Y path, without another global search.
        self.mode('stop')
        base=self.map_base_pose()
        dx,dy=relative(goal,base)
        if abs(dx)>.025 or not 0<side*dy<=.251:return None
        if not self.fresh('scan',.5) or not self.fresh('costmap',3.):return None
        grid=self.last['costmap']
        if grid.header.frame_id!='map':return None
        size,center_x=self.travel_envelope()
        center=rack_pose(base,(center_x,0.))
        # Reuse the rear-bumper sweep with its -X axis pointing toward the side step.
        strip=(center[0],center[1],base[2]-side*math.pi/2)
        blocked=rear_strip_blockage(grid,strip,-abs(dy),(size[1],size[0]))
        if blocked:
            print('横移通道地图检查：'+blocked.replace('车尾','侧向'),flush=True)
            return None
        try:
            points,origin=self.scan_points(self.last['scan'],'base_link')
            if self.carrying:
                points=[p for p in points if all(math.dist(p,leg)>.026 for leg in self.payload_legs)]
            check_detour_scan(points,origin,side*math.pi/2,0.,abs(dy),size,pure_strafe=True,center_x=center_x)
        except (TransformException,ValueError,RuntimeError) as error:
            print(f'横移通道扫描检查：{error}',flush=True)
            return None
        path=Path()
        path.header.frame_id='map'
        path.header.stamp=self.get_clock().now().to_msg()
        steps=max(1,math.ceil(abs(dy)/grid.info.resolution))
        for i in range(steps+1):
            p=PoseStamped();p.header=path.header
            p.pose.position.x,p.pose.position.y=world((0.,dy*i/steps),base)
            p.pose.orientation.z=math.sin(base[2]/2);p.pose.orientation.w=math.cos(base[2]/2)
            path.poses.append(p)
        request=FollowPath.Goal()
        request.path=path
        request.controller_id='StrafeLeft' if side>0 else 'StrafeRight'
        return request

    def navigate(self,pose,*,alternate_on_block=False,stop_before=None,strafe=0):
        label=('左移' if strafe>0 else '右移') if strafe else '导航'
        blocked_attempts=0
        while True:
            self.wait_for_devices()
            if stop_before and stop_before():
                self.mode('stop')
                return False
            self.report_route('规划中',action=label,goal=pose)
            generation=self.reconnections
            req=NavigateToPose.Goal()
            req.pose.header.frame_id='map';req.pose.header.stamp=self.get_clock().now().to_msg()
            req.pose.pose.position.x,req.pose.pose.position.y=pose[:2]
            req.pose.pose.orientation.z=math.sin(pose[2]/2);req.pose.pose.orientation.w=math.cos(pose[2]/2)
            if self.fixed:
                req=self.strafe_request(pose,strafe) if strafe else self.checked_route(req.pose)
                if req is None:
                    self.report_route('没有可用路径')
                    if strafe:return False
                    if generation!=self.reconnections:continue
                    blocked_attempts+=1
                    if alternate_on_block and blocked_attempts>=2:return False
                    self.pause(1.)
                    continue
                if strafe:
                    odom_start=self.last['odom'].pose.pose
            if self.reconnections!=generation:
                if strafe:return False
                continue
            if strafe:
                self.strafe_peaks={}
                print(f'提交横移控制器 {req.controller_id}',flush=True)
            self.preview_follow(label,pose,req)
            self.nav_goal=self.future(self.nav.send_goal_async(req),4,'导航请求超时')
            if not self.nav_goal.accepted:
                self.nav_goal=None
                if generation!=self.reconnections:continue
                self.report_route('目标被拒绝')
                raise RuntimeError('导航目标被拒绝')
            if generation==self.reconnections:
                self.mode('nav')
            if generation!=self.reconnections:
                self.nav_goal.cancel_goal_async()
            pending=self.nav_goal.get_result_async()
            if stop_before:
                self.wait(lambda:pending.done() or stop_before(),None,'等待导航结果或分段停车条件')
                if not pending.done():
                    self.mode('stop')
                    self.nav_goal.cancel_goal_async()
                    self.future(pending,4,'等待导航取消')
                    self.report_route('已停止')
                    self.nav_goal=None
                    return False
            result=self.future(pending,None,'导航等待')
            self.nav_goal=None
            self.mode('stop')
            if generation!=self.reconnections:
                if strafe:return False
                continue
            self.report_route('跟踪结束' if result.status==4 else f'受阻 {result.result.error_code}')
            if strafe:
                odom_end=self.last['odom'].pose.pose
                q=odom_start.orientation
                start_yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
                dx,dy=relative((odom_end.position.x,odom_end.position.y),
                               (odom_start.position.x,odom_start.position.y,start_yaw))
                print(f'横移实测 前后 {dx:+.3f} m，左右 {dy:+.3f} m，控制器返回 {result.result.error_code}',flush=True)
                if result.status==4:
                    return (dy*strafe>0 and abs(dx)<=.04
                            and math.dist(self.map_base_pose()[:2],pose[:2])<=NAV_GOAL_TOLERANCE_M)
            if generation!=self.reconnections:continue
            if result.status!=4:
                code=result.result.error_code
                if result.status==6 and code in (FollowPath.Result.FAILED_TO_MAKE_PROGRESS,
                        FollowPath.Result.NO_VALID_CONTROL,FollowPath.Result.PATIENCE_EXCEEDED,
                        ComputePathToPose.Result.NO_VALID_PATH,ComputePathToPose.Result.START_OCCUPIED,
                        ComputePathToPose.Result.GOAL_OCCUPIED):
                    blocked_attempts+=1
                    if blocked_attempts==1:self.report_nav_block(pose)
                    if strafe:return False
                    if alternate_on_block and (code==FollowPath.Result.PATIENCE_EXCEEDED or blocked_attempts>=2):
                        print(f'当前目标受阻 ({code})，已停车并切换观察位或脱困路线',flush=True)
                        return False
                    print(f'Nav2 受阻 ({code})，已停车，保留目标重新规划',flush=True)
                    self.pause(1.)
                    continue
                raise RuntimeError(f'Nav2 导航失败 status={result.status}, error_code={code}')
            return True

    def report_nav_block(self,goal):
        grid=self.last.get('costmap')
        if grid is not None and self.fresh('costmap',3.) and grid.header.frame_id=='map' and self.last_map_pose:
            cells=[grid_cell(grid,p) for p in (self.last_map_pose,goal)]
            costs=[grid.data[i] if i is not None else None for i in cells]
            print(f'受阻时全局代价值：最近定位格 {costs[0]}，目标格 {costs[1]}（100 占用，-1 未知）',flush=True)
        if self.fresh('scan',.6):
            try:points,_=self.scan_points(self.last['scan'],'base_link')
            except TransformException:return
            if points:
                x,y=min(points,key=lambda p:math.hypot(*p))
                print(f'同期最近过滤回波：距车心 {math.hypot(x,y):.2f} m，前向 {x:+.2f}，左向 {y:+.2f} m',flush=True)

    def face_rack_at_gate(self,rack):
        gate=self.centerline.rack_gate(rack['approach'])
        print(f"中轴预对正 {rack['code']}：目标车头 {math.degrees(gate[2]):.1f}°",flush=True)
        self.follow_centerline(gate)
        heading=self.map_base_pose()[2]
        error=math.degrees(math.remainder(gate[2]-heading,math.tau))
        print(f'中轴到位：实测车头 {math.degrees(heading):.1f}°，剩余偏差 {error:.1f}°',flush=True)
        return gate

    def follow_centerline(self, goal):
        self.phase('centerline')
        self.navigate_transit(goal,centerline=True)

    def navigate_transit(self,goal,*,centerline=False):
        def close_enough(base):
            return (math.dist(base[:2],goal[:2])<=NAV_GOAL_TOLERANCE_M
                    and abs(math.remainder(goal[2]-base[2],math.tau))<=AISLE_HEADING_TOLERANCE)
        def stop_at_goal():
            nonlocal arrival_requested
            arrival_requested=close_enough(self.map_base_pose())
            return arrival_requested
        while True:
            arrival_requested=False
            reached=self.navigate(goal,alternate_on_block=True,
                                  stop_before=stop_at_goal if centerline else None)
            if reached and not centerline:return
            base=self.map_base_pose()
            error=math.dist(base[:2],goal[:2])
            if centerline:
                yaw_error=math.remainder(goal[2]-base[2],math.tau)
                print(f'中轴目标 ({goal[0]:.3f}, {goal[1]:.3f})，停车定位 ({base[0]:.3f}, {base[1]:.3f})，'
                      f'横向偏差 {base[1]-goal[1]:+.3f} m，到点误差 {error:.3f} m，朝向偏差 {math.degrees(yaw_error):+.1f}°',flush=True)
                if close_enough(base):return
                if ((arrival_requested or reached) and abs(yaw_error)<=AISLE_HEADING_TOLERANCE
                        and max(abs(base[i]-goal[i]) for i in (0,1))<=NAV_GOAL_TOLERANCE_M):
                    return
                if arrival_requested:
                    print('到点刹停后偏出容差，重新靠近原中轴目标',flush=True)
            if not reached and not arrival_requested:
                if error>NAV_GOAL_TOLERANCE_M and self.try_side_detour(goal):continue
                print('导航死路，检查车后 0.15 m 退路，保留当前目标',flush=True)
                self.checked_backup(-.15,'导航脱困',include_target=True)
                print('保留任务，从当前停车位置重新规划原目标',flush=True)
            self.pause(.25 if reached else 1.)

    def try_side_detour(self,goal):
        start=self.map_base_pose()
        remaining=math.dist(start[:2],goal[:2])
        step=min(.25,remaining)
        dx,dy=relative(goal,start)
        side=1 if dy>=0 else -1
        lengths=(step,.12) if self.fixed and step>.12+NAV_GOAL_TOLERANCE_M else (step,)
        options=[(angle,0.,length,label,self.fixed)
                 for angle,label in ((math.pi/2,'向左横移'),(-math.pi/2,'向右横移'))
                 for length in lengths]
        if remaining>step+NAV_GOAL_TOLERANCE_M:
            options.append((math.atan2(dy,dx),0.,min(.12,step),'向原目标短移',False))
        for sign in (side,-side):
            angle=sign*math.pi/4
            options.append((angle,angle,step,'左前转向' if sign>0 else '右前转向',False))
        for angle,turn,travel,label,pure_strafe in options:
            self.wait_for_devices()
            base=self.map_base_pose()
            if math.dist(base[:2],start[:2])>NAV_GOAL_TOLERANCE_M:return False
            if not self.fresh('scan',.5) or not self.fresh('costmap',3.):return False
            path_angle=base[2]+angle
            via=(base[0]+travel*math.cos(path_angle),base[1]+travel*math.sin(path_angle),
                 math.remainder(base[2]+turn,math.tau))
            grid=self.last['costmap']
            if grid.header.frame_id!='map':return False
            cell=grid_cell(grid,via)
            if cell is None or not 0<=grid.data[cell]<(100 if pure_strafe else 99):continue
            try:
                points,origin=self.scan_points(self.last['scan'],'base_link')
            except TransformException:return False
            if self.carrying:
                points=[p for p in points if all(math.dist(p,leg)>.026 for leg in self.payload_legs)]
            try:
                size,center_x=self.travel_envelope()
                check_detour_scan(points,origin,angle,turn,travel,size,pure_strafe=pure_strafe,center_x=center_x)
            except (ValueError,RuntimeError) as reason:
                print(f'{label} {travel:.2f} m 未采用：{reason}',flush=True)
                continue
            print(f'雷达通道可见，导航尝试{label} {travel:.2f} m',flush=True)
            if pure_strafe:
                signals=(('控制器','/cmd_vel_nav'),('平滑后','/cmd_vel_smoothed'),('避碰后','/nav_cmd_vel'),
                         ('底盘指令','/cmd_vel'),('底盘反馈','chassis'))
                subscriptions=[]
                try:
                    for _,topic in signals[:-1]:
                        subscriptions.append(self.create_subscription(Twist,topic,
                            lambda msg,k=topic:self.record_strafe_vy(k,msg.linear.y),qos_profile_sensor_data))
                    if self.navigate(via,alternate_on_block=True,strafe=1 if angle>0 else -1):return True
                finally:
                    peaks=self.strafe_peaks
                    self.strafe_peaks=None
                    for sub in subscriptions:self.destroy_subscription(sub)
                    if peaks is not None:
                        print('横移 vy 峰值 (cm/s，左正右负)：'+'，'.join(
                            f'{name} {peaks[key]*100:+.1f}' if key in peaks else f'{name} 未采到'
                            for name,key in signals),flush=True)
            else:
                odom=self.last['odom'].pose.pose
                if self.navigate(via,alternate_on_block=True):
                    end=self.last['odom'].pose.pose.position
                    q=odom.orientation
                    heading=math.atan2(2*q.w*q.z,1-2*q.z*q.z)+angle
                    along=(end.x-odom.position.x)*math.cos(heading)+(end.y-odom.position.y)*math.sin(heading)
                    if along>max(.01,travel-NAV_GOAL_TOLERANCE_M) and math.dist(self.map_base_pose()[:2],via[:2])<=NAV_GOAL_TOLERANCE_M:
                        return True
        print('侧向候选均未通过雷达、地图或导航检查，保留原目标',flush=True)
        return False

    def leave_start(self):
        self.mode('stop')
        self.phase('approach')
        delta=math.remainder(math.pi-self.map_base_pose()[2],math.tau)
        if abs(delta)>AISLE_HEADING_TOLERANCE:
            print(f'START 已核码，就地转向终点墙，剩余 {math.degrees(delta):+.1f}°',flush=True)
            self.wait(self.turn.server_is_ready,5,'等待 Nav2 原地转向服务')
            if self.spin_by(delta) is None:
                raise RuntimeError('Nav2 拒绝原地转向')
        for goal in start_passage(self.c):
            print(f'START 绕过 P4，通道目标 ({goal[0]:.3f}, {goal[1]:.3f})',flush=True)
            self.navigate_transit(goal)

    def spin_by(self,delta):
        self.relative_preview('原地转向',turn=delta)
        req=Spin.Goal()
        req.target_yaw=delta
        req.time_allowance.sec=15
        generation=self.reconnections
        self.nav_goal=self.future(self.turn.send_goal_async(req),4,'转向请求超时')
        if not self.nav_goal.accepted:
            self.nav_goal=None
            print('Nav2 拒绝原地转向，保持停车',flush=True)
            return None
        if generation==self.reconnections:self.mode('nav')
        if generation!=self.reconnections:self.nav_goal.cancel_goal_async()
        self.report_route('已提交')
        result=self.future(self.nav_goal.get_result_async(),None,'等待原地转向')
        self.report_route('跟踪结束' if result.status==4 else f'受阻 {result.result.error_code}')
        self.nav_goal=None
        self.mode('stop')
        if result.status!=4:
            print(f'原地转向未完成，Nav2 返回 {result.result.error_code}',flush=True)
        return generation==self.reconnections and result.status==4

    def scan(self,expected,*,try_another_view=False,quick=False):
        self.mode('stop')
        self.report_route('等待扫码',action=expected)
        self.pause(.35)
        attempts=0
        while attempts<(1 if try_another_view else 2):
            generation=self.reconnections
            token=secrets.token_hex(8)
            started=self.get_clock().now().nanoseconds
            self.phase('scan', expected=expected)
            self.send('/match/scan_target',String(data=json.dumps(dict(id=token,expected=expected,quick_rack_scan=quick))))
            print('停车等候二维码 '+expected,flush=True)
            def completed():
                event=self.last.get('vision',{})
                return (event.get('request_id')==token and event.get('expected')==expected
                        and event.get('state') in ('SUCCEEDED','TIMEOUT','ERROR'))
            self.wait(lambda:generation!=self.reconnections or completed(),12,'扫码节点未确认 '+expected)
            if generation!=self.reconnections:continue
            attempts+=1
            event=self.last['vision']
            if try_another_view and event['state']=='TIMEOUT' and event.get('frames')==0:
                print('没有新的相机帧，停车等待后重扫',flush=True)
                attempts=0
                quick=False
                continue
            if event['state']=='SUCCEEDED':
                stamp=event.get('stamp_sec',0)*10**9+event.get('stamp_nanosec',0)
                if event.get('text')==expected and started<=stamp<=self.get_clock().now().nanoseconds:
                    print('扫码确认 '+expected,flush=True)
                    return True
            if event['state']=='ERROR':
                break
        else:
            if try_another_view:
                print(f'架外未读到 {expected}',flush=True)
                return False
        raise RuntimeError('扫码未确认 '+expected)

    def publish_keepout(self,*,exiting=None):
        current=self.last['map']
        overlay=OccupancyGrid()
        overlay.header.frame_id='map'
        overlay.header.stamp=self.get_clock().now().to_msg()
        overlay.info=current.info
        overlay.data=current.data[:]
        posts=self.c['qr_posts']
        racks=[*self.standing.values(),*self.deposited]
        legs=[]
        if exiting is not None:
            rack,pose=exiting
            racks=self.other_racks(undocking=rack['code'] not in self.standing)
            # Inside the opening, represent this rack by its four supports.
            spacing=rack['observed_spacing']
            radius=math.hypot(*(max(0.,outer-gap)/2 for outer,gap in zip(self.c['rack_size_m'],spacing)))
            legs=[(*world(p,pose),radius) for p in rack['leg_offsets']]
            for i in keepout_cells(overlay,legs,[],self.c['rack_size_m'],margin=0.):
                overlay.data[i]=100
        paint_keepout(overlay,posts,racks,self.c['rack_size_m'],.02 if self.nav_only else MARGIN)
        local=local_structure_map(current,posts+legs,racks,self.c['rack_size_m'])
        local.header=overlay.header
        self.send(self.local_structure_topic,local)
        self.send(self.keepout_topic,overlay)
        after=time.monotonic()
        points=posts+racks+legs
        def acknowledged():
            grid=self.last.get('costmap')
            if grid is None or grid.header.frame_id!='map' or self.receipt['costmap']<=after:
                return False
            indices=[grid_cell(grid,point) for point in points]
            return all(i is not None and grid.data[i]>=99 for i in indices)
        self.wait(acknowledged,5,'全局代价地图未确认立柱或已落架障碍')

    def scan_points(self, scan, frame):
        stamp=Time.from_msg(scan.header.stamp)
        t=self.tf.lookup_transform(frame,scan.header.frame_id,stamp).transform
        yaw=math.atan2(2*t.rotation.w*t.rotation.z,1-2*t.rotation.z**2)
        origin=(t.translation.x,t.translation.y)
        points=[]
        for i,r in enumerate(scan.ranges):
            if math.isfinite(r) and scan.range_min<=r<scan.range_max:
                a=yaw+scan.angle_min+i*scan.angle_increment
                points.append((origin[0]+r*math.cos(a),origin[1]+r*math.sin(a)))
        return points,origin

    def check_scan_corridor(self,meters):
        points,origin=self.scan_points(self.last['scan'],'base_link')
        check_corridor(points,origin,meters,self.c['base_size_m'],self.c.get('body_center_x_m',0.))

    def locate_rack(self,rack,retry=False):
        views=[rack['approach'],*rack['scan_views']]
        if retry: views=views[1:]+views[:1]
        while True:
            try:
                if self.survey_views(rack,views):return
            except RackViewLost as error:
                print(str(error),flush=True)
            self.mode('stop')
            print(f"{rack['code']} 架外观察未完成，停车后重试同一货架",flush=True)
            self.pause(2.)

    def rack_scan_backoff(self,rack,pose):
        """Signed straight reverse needed at the mouth; None means unsafe heading."""
        base=self.map_base_pose()
        forward,side=relative(base,pose)
        stand=-relative(rack['approach'],rack['rack_hint'])[0]
        if abs(side)>self.c['rack_size_m'][1]/2+self.c['radius_empty_m'] or forward < -stand+NAV_GOAL_TOLERANCE_M:
            return 0.
        if forward>self.c['rack_size_m'][0]/2+self.c['radius_empty_m']:
            return 0.  # Beyond the opposite side, let Nav2 go around the rack.
        projection=math.cos(base[2]-pose[2])
        if projection<math.cos(math.radians(30)):
            return None
        # A reverse along base -X must increase the distance to the rack face.
        return -(stand+forward)/projection

    def backup_for_qr(self,rack,distance):
        return self.checked_backup(distance,rack['code']+' 扫码')

    def travel_envelope(self):
        size=self.c['base_size_m']
        center_x=self.c.get('body_center_x_m',0.)
        if self.carrying:
            length,width=size
            rx,ry=self.c['load_center_m']
            rl,rw=self.c['rack_size_m']
            rear=min(center_x-length/2,rx-rl/2)
            front=max(center_x+length/2,rx+rl/2)
            center_x=(front+rear)/2
            size=(front-rear,max(width,rw+2*abs(ry)))
        return size,center_x

    def checked_backup(self,distance,label,*,include_target=False):
        """Reverse along body -X after checking the whole base or carried envelope."""
        self.mode('stop')
        self.report_route('检查退路',action=label+'倒车')
        if not -.20<=distance<0 or not self.fresh('scan',.5) or not self.fresh('costmap',3.):
            print(f'{label}倒车等待新扫描、地图或可用短退距离',flush=True)
            return False
        base=self.map_base_pose()
        grid=self.last['costmap']
        size,center_x=self.travel_envelope()
        if grid.header.frame_id!='map':return False
        occupied=[*self.standing.values(),*self.deposited] if include_target else self.other_racks()
        try:
            verify_retreat(base,distance,dict(self.c,base_size_m=size,body_center_x_m=center_x),occupied,allow_departure=True)
        except ValueError as error:
            print(f'{label}倒车受阻：{error}',flush=True)
            return False
        static=self.last.get('map')
        def registered(point):
            # Reproduce the overlay at its original cell centre, only on known free floor.
            if static is None:return False
            i=grid_cell(static,point)
            if i is None or static.data[i]!=0:return False
            info=static.info
            o=info.origin
            yaw=math.atan2(2*o.orientation.w*o.orientation.z,1-2*o.orientation.z**2)
            p=world((((i%info.width)+.5)*info.resolution,((i//info.width)+.5)*info.resolution),
                    (o.position.x,o.position.y,yaw))
            extra=info.resolution/math.sqrt(2)
            return in_keepout(p,self.c['qr_posts'],occupied,self.c['rack_size_m'],extra)
        blocked=rear_strip_blockage(grid,rack_pose(base,(center_x,0.)),distance,size,registered)
        if blocked:
            print(f'{label}倒车：{blocked}，保持停车',flush=True)
            return False
        try:
            points,origin=self.scan_points(self.last['scan'],'base_link')
        except TransformException:return False
        if not points:
            print(f'{label}倒车：本帧没有有效回波，保持停车',flush=True)
            return False
        try:
            check_corridor(points,origin,distance,size,center_x)
        except IncompleteCoverage as error:
            print(f'{label}倒车：{error}，使用已核查的地图通道和 Nav2 后退避碰',flush=True)
        except (ValueError,RuntimeError) as error:
            print(f'{label}倒车受阻：{error}',flush=True)
            return False
        if not self.backup_client.server_is_ready():
            print('Nav2 倒车服务未就绪，保持停车',flush=True)
            return False
        start=self.last['odom'].pose.pose
        generation=self.reconnections
        self.relative_preview(label+'倒车',meters=distance)
        req=BackUp.Goal()
        req.target.x=distance
        req.speed=.06
        req.time_allowance.sec=10
        self.nav_goal=self.future(self.backup_client.send_goal_async(req),4,f'{label}倒车请求超时')
        if not self.nav_goal.accepted:
            self.nav_goal=None
            print(f'Nav2 拒绝{label}倒车',flush=True)
            return False
        if generation==self.reconnections:self.mode('nav')
        if generation!=self.reconnections:self.nav_goal.cancel_goal_async()
        self.report_route('已提交')
        print(f"{label}倒车目标 {distance:+.3f} m",flush=True)
        result=self.future(self.nav_goal.get_result_async(),None,f'等待{label}倒车')
        self.report_route('跟踪结束' if result.status==4 else f'受阻 {result.result.error_code}')
        self.nav_goal=None
        self.mode('stop')
        if generation!=self.reconnections:return False
        end=self.last['odom'].pose.pose
        def pose2d(p):
            q=p.orientation
            return p.position.x,p.position.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)
        origin=pose2d(start)
        final=pose2d(end)
        dx,dy=relative(final,origin)
        angle=math.remainder(final[2]-origin[2],math.tau)
        print(f'{label}倒车实测 前后 {dx:+.3f} m，横移 {dy:+.3f} m，转角 {math.degrees(angle):+.1f}°，Nav2 返回 {result.result.error_code}',flush=True)
        return (result.status==4 and dx<0 and abs(dx-distance)<=.025 and
                abs(dy)<=.025 and abs(angle)<=math.radians(5))

    def recover_rack_heading(self,rack,pose):
        self.mode('stop')
        for attempt in range(2):
            if not self.fresh('scan',.5) or not self.fresh('costmap',3.) or not self.turn.server_is_ready():
                return False
            base=self.map_base_pose()
            delta=math.remainder(pose[2]-base[2],math.tau)
            if abs(delta)<=math.radians(30):return True
            # Keep the rotating corners clear of the observed rack as well as Nav2's costmap.
            if box_distance(base,pose,self.c['rack_size_m'])>self.c['radius_empty_m']+.01:
                print(f"{rack['code']} 架口回正，目标转角 {math.degrees(delta):+.1f}°",flush=True)
                generation=self.reconnections
                self.spin_by(delta)
                base=self.map_base_pose()
                if self.reconnections!=generation:return False
                delta=math.remainder(pose[2]-base[2],math.tau)
                print(f'架口回正后，定位角度偏差 {math.degrees(delta):+.1f}°',flush=True)
                if abs(delta)<=math.radians(30):return True
            if attempt:return False
            # Retreat only when the full base is outside the entry face and -X moves away.
            forward,_=relative(base,pose)
            length,width=self.c['base_size_m']
            front=forward+self.c.get('body_center_x_m',0.)*math.cos(delta)+(abs(math.cos(delta))*length+abs(math.sin(delta))*width)/2
            if front>=-self.c['rack_size_m'][0]/2 or math.cos(delta)<.5:return False
            if not self.backup_for_qr(rack,-.08):return False

    def prepare_rack_scan(self,rack,pose,reached):
        distance=self.rack_scan_backoff(rack,pose)
        if distance is None:
            if not self.recover_rack_heading(rack,pose):
                raise RackViewLost('架口偏斜，回正或后退未获确认，保持停车复查本架')
            distance=self.rack_scan_backoff(rack,pose)
            if distance is None:raise RackViewLost('架口朝向仍未确认，保持停车复查本架')
            reached=True
        if distance<0:
            if not self.backup_for_qr(rack,distance):return False
            if self.rack_scan_backoff(rack,pose)!=0.:return False
        elif not reached:return False
        base=self.map_base_pose()
        yaw=math.atan2(pose[1]-base[1],pose[0]-base[0])
        return relative(base,pose)[0]<0 and abs(math.remainder(yaw-base[2],math.tau))<math.radians(30)

    def survey_views(self,rack,views):
        hint=rack['rack_hint']
        views=list(views)
        farther_view=None
        captures=[]
        confirmed=False
        for index,view in enumerate(views,1):
            self.phase('survey')
            yaw=math.atan2(hint[1]-view[1],hint[0]-view[0])
            print(f"{rack['code']} 架外观察目标 {index}/{len(views)}：({view[0]:.2f}, {view[1]:.2f})",flush=True)
            pose=self.standing[rack['code']]
            reached=self.navigate((*view[:2],yaw),alternate_on_block=True,
                                  stop_before=lambda:self.rack_scan_backoff(rack,pose)!=0.)
            if not reached and self.rack_scan_backoff(rack,pose)==0.:
                if index==len(views) and math.dist(self.map_base_pose()[:2],pose[:2])>math.dist(view[:2],hint[:2])+NAV_GOAL_TOLERANCE_M:
                    self.try_side_detour((*view[:2],yaw))
                continue
            last_stamp=self.last['scan'].header.stamp
            for _ in range(3):
                def next_scan():
                    s=self.last['scan']
                    return (s.header.stamp!=last_stamp and self.fresh('scan',.5)
                            and self.tf.can_transform('map',s.header.frame_id,Time.from_msg(s.header.stamp)))
                self.wait(next_scan,3,'等待停车后的新雷达扫描及地图坐标')
                scan=self.last['scan']
                last_stamp=scan.header.stamp
                points,_=self.scan_points(scan,'map')
                captures.append(points)
            legs=stable_legs(captures)
            spacing=self.c.get('leg_spacing_m') or measure_spacing(legs,hint,self.c['rack_size_m'],self.c['base_size_m'][1])
            result=fit_rack(legs,hint,spacing) if spacing else None
            if result is not None:
                pose,legs_for_target=result
                size=self.c['rack_size_m']
                if any(overlap(pose,size,other,size) for other in self.other_racks()):
                    print('四腿拟合位置与其他已登记货架过近，丢弃本次观测并换视角',flush=True)
                    captures.clear()
                    confirmed=False
                    continue
                if view==farther_view:view=rack['approach']
                local=relative(view,hint)
                if not 0 < -local[0]-self.c['load_center_m'][0] <= rack['max_insert_m']:
                    view=rack['approach']
                    local=relative(view,hint)
                heading=math.remainder(view[2]+pose[2]-hint[2],math.tau)
                goal=(*rack_pose(pose,local)[:2],heading)
                grid=self.last['map']
                cell=grid_cell(grid,goal)
                if cell is None or not 0<=grid.data[cell]<25:
                    print('四腿修正后的接近点不在已知空闲区，丢弃本次观测并换视角',flush=True)
                    captures.clear()
                    confirmed=False
                    continue
            if not self.prepare_rack_scan(rack,result[0] if result else pose,reached):continue
            if not confirmed:
                confirmed=self.scan(rack['code'],try_another_view=True,quick=True)
                if not confirmed and self.fixed:
                    base=self.map_base_pose()
                    left=(*world((0.,.08),base),base[2])
                    print(f"{rack['code']} 架外未读到码，保持朝向尝试向车身左侧横移 8 cm",flush=True)
                    if self.navigate(left,alternate_on_block=True,strafe=1):
                        confirmed=self.scan(rack['code'],try_another_view=True)
                    self.strafe_peaks=None
                if not confirmed and self.backup_for_qr(rack,-.08):
                    confirmed=self.scan(rack['code'],try_another_view=True)
                if not confirmed:
                    if self.centerline and farther_view is None:
                        x,y,heading=rack['approach']
                        left=(-math.sin(heading),math.cos(heading))
                        offset=max(-.24,min(.24,(self.centerline.y-y)*left[1]))
                        farther_view=(x-.08*math.cos(heading)+offset*left[0],
                                      y-.08*math.sin(heading)+offset*left[1],heading)
                        views.insert(index,farther_view)
                        print(f'改查靠中轴的远侧扫码位 ({farther_view[0]:.2f}, {farther_view[1]:.2f})',flush=True)
                    continue
            if result is not None:
                self.target_legs=legs_for_target
                rack['observed_spacing']=spacing
                self.standing[rack['code']]=pose
                self.publish_keepout()
                print('雷达确认四条架腿 '+rack['code']+' '+json.dumps(self.target_legs),flush=True)
                self.phase('approach')
                if self.navigate(goal,alternate_on_block=True):return True
        return False

    def check_exit_clearance(self,base,distance,occupied):
        # The chassis exits alone, including just after the rack is lowered.
        try:
            verify_retreat(base,distance,self.c,occupied)
        except ValueError as error:
            raise RetreatBlocked(str(error)) from error
        try:
            points,origin=self.scan_points(self.last['scan'],'base_link')
            if not points or not self.fresh('scan',.5):
                raise RetreatBlocked('等待新的有效退架扫描')
            check_corridor(points,origin,distance,self.c['base_size_m'],self.c.get('body_center_x_m',0.))
        except IncompleteCoverage:
            grid=self.last.get('costmap')
            if grid is None or grid.header.frame_id!='map' or not self.fresh('costmap',3.):
                raise RetreatBlocked('后方覆盖不足，等待新代价地图')
            center=rack_pose(base,(self.c.get('body_center_x_m',0.),0.))
            blocked=rear_strip_blockage(grid,center,distance,self.c['base_size_m'])
            if blocked:raise RetreatBlocked(blocked)
        except TransformException as error:
            raise RetreatBlocked('等待扫描坐标变换') from error
        except RetreatBlocked:
            raise
        except RuntimeError as error:
            raise RetreatBlocked(str(error)) from error

    def check_relative(self):
        generation=self.reconnections
        start,yaw,meters=self.relative_path
        p=self.last['odom'].pose.pose.position
        traveled=(p.x-start[0])*math.cos(yaw)+(p.y-start[1])*math.sin(yaw)
        left=math.copysign(max(0.,abs(meters)-math.copysign(1.,meters)*traveled),meters)
        base=self.map_base_pose()
        if self.reconnections!=generation:return
        occupied=self.other_racks(undocking=meters<0 and self.current_rack not in self.standing)
        if meters<0:
            if left<-.001:self.check_exit_clearance(base,left,occupied)
        else:
            verify_retreat(base,left,self.c,occupied)
            self.check_scan_corridor(left)
        self.last_guard_scan=self.last['scan'].header.stamp

    def map_base_pose(self):
        # Use the fresh odometry sample's time; AMCL can future-date its map transform.
        until=self.work_time()+.10
        while True:
            self.wait_for_devices()
            try:
                when=Time.from_msg(self.last['odom'].header.stamp)
                pose=self.tf.lookup_transform('map','base_link',when)
                break
            except TransformException:
                pass
            if self.work_time()>=until:
                if not self.wait_links:
                    raise RuntimeError('地图定位过期，停车')
                self.wait_for_devices('地图定位过期')
                until=self.work_time()+.10
            self.poll()
        t=pose.transform
        yaw=math.atan2(2*t.rotation.w*t.rotation.z,1-2*t.rotation.z**2)
        self.last_map_pose=(t.translation.x,t.translation.y,yaw)
        return self.last_map_pose

    def other_racks(self, *, undocking=False):
        standing=[pose for code,pose in self.standing.items() if code!=self.current_rack]
        placed=self.deposited[:-1] if undocking else self.deposited
        return [*standing,*placed]

    def check_alignment_travel(self):
        base=self.map_base_pose()
        cs=self.last['chassis']
        measured=(cs.vx_mps,cs.vy_mps,cs.wz_rad_s)
        commanded=tuple(self.last['gate'].get('velocity',(0.,0.,0.)))
        racks=self.other_racks()
        radius=self.c['radius_loaded_m' if self.carrying else 'radius_empty_m']
        points,_=self.scan_points(self.last['scan'],'base_link')
        body=(self.c.get('body_center_x_m',0.),0.,0.)
        rack=(*self.c['load_center_m'],0.)
        external=[p for p in points if box_distance(p,body,self.c['base_size_m'])>0
                  and (not self.carrying or box_distance(p,rack,self.c['rack_size_m'])>0)]
        for velocity in {measured,commanded}:
            verify_travel(base,velocity,radius,self.last['map'],self.c['qr_posts'],racks,self.c['rack_size_m'],
                          scan=external,body_size=self.c['base_size_m'],body_center_x=body[0],
                          scan_radius=None if self.carrying else self.c.get('empty_scan_stop_m'))
        self.last_travel_scan=self.last['scan'].header.stamp

    def drop_grid(self):
        if (not self.fresh('costmap',3.) or self.last['costmap'].header.frame_id!='map'
                or self.last['costmap'].info.resolution<=0):
            raise RuntimeError('等待新鲜的全局代价地图，停止卸货')
        return obstacle_grid(self.last['costmap'])

    def reserved_goals(self):
        reserved=[(self.c['end_approach'],self.c['radius_empty_m'])]
        for rack in self.c['racks']:
            if rack['code'] in self.completed or rack['code']==self.current_rack:
                continue
            reserved.append((rack['approach'],self.c['radius_empty_m']))
            if rack['slot'] is not None:
                reserved.append((rack['slot'],self.c['radius_loaded_m']))
        if self.centerline:
            reserved.append(((self.centerline.end_x,self.centerline.y),self.c['radius_loaded_m']))
        return reserved

    def choose_drop(self):
        self.mode('stop')
        self.wait(self.planner.server_is_ready,3,'卸货选位等待 Nav2 规划器超时')
        options=candidates(self.c,self.other_racks(),self.drop_grid(),self.reserved_goals(),self.spin)
        for slot in options:
            req=ComputePathToPose.Goal()
            req.goal.header.frame_id='map'
            req.goal.header.stamp=self.get_clock().now().to_msg()
            req.goal.pose.position.x,req.goal.pose.position.y=slot[:2]
            req.goal.pose.orientation.z=math.sin(slot[2]/2)
            req.goal.pose.orientation.w=math.cos(slot[2]/2)
            req.planner_id='GridBased'
            handle=self.future(self.planner.send_goal_async(req),3,'卸货路径请求超时')
            if handle.accepted:
                reply=self.future(handle.get_result_async(),4,'卸货路径规划超时')
                if reply.status==4 and reply.result.path.poses:
                    end=reply.result.path.poses[-1].pose.position
                    if math.dist((end.x,end.y),slot[:2])<=.06:
                        print('终点区自动选位',slot,flush=True)
                        return slot
        raise RuntimeError('终点区没有找到可达且能倒退出架的空位')

    def measured_drop(self,slot):
        base=self.map_base_pose()
        yaw=base[2]
        if math.dist(base[:2],slot[:2])>.06 or abs(math.remainder(yaw-slot[2],math.tau))>.12:
            raise RuntimeError('实际位姿偏离放置点，停止落架')
        actual=rack_pose(base,self.c['load_center_m'])
        verify_drop(self.c,actual,self.other_racks())
        return actual

    def align(self,rack):
        self.report_route('雷达精对准',action='对准架口')
        self.phase('align')
        code=rack['code']
        self.check_alignment_travel()
        self.mode('lidar')
        end=self.work_time()+18
        generation=self.reconnections
        stable=0
        stamp=None
        try:
            while self.work_time()<end:
                self.spin()
                if generation!=self.reconnections or self.lease!='lidar':
                    generation=self.reconnections
                    stable=0
                    self.check_alignment_travel()
                    self.mode('lidar')
                    continue
                scan=self.last['scan']
                if scan.header.stamp==stamp:
                    continue
                when=Time.from_msg(scan.header.stamp)
                self.send('/lidar/cmd_vel',Twist())
                if not self.fresh('scan',.25) or not self.tf.can_transform('map','base_link',when):
                    stable=0
                    continue
                stamp=scan.header.stamp
                t=self.tf.lookup_transform('map','base_link',when).transform
                base=(t.translation.x,t.translation.y,math.atan2(2*t.rotation.w*t.rotation.z,1-2*t.rotation.z**2))
                saved=self.standing[code]
                hint=(*relative(saved,base),math.remainder(saved[2]-base[2],math.tau))
                points,origin=self.scan_points(scan,'base_link')
                fit=fit_rack(leg_centers(points),hint,rack['observed_spacing'])
                if fit is None:
                    stable=0
                    continue
                pose,measured_legs=fit
                dx,dy,yaw=insertion_error(pose,self.c['load_center_m'])
                if not 0<dx<=rack['max_insert_m']:
                    raise RackViewLost('雷达测得的进架距离超出标定上限')
                measured=(*rack_pose(base,pose[:2])[:2],math.remainder(base[2]+pose[2],math.tau))
                if math.dist(measured[:2],saved[:2])>.06:
                    raise RackViewLost('精对准架位与已核码货架不一致')
                length,width=self.c['base_size_m']
                swept=2*abs(dy)+width*abs(math.cos(yaw))+length*abs(math.sin(yaw))+2*dx*abs(math.sin(yaw))
                cmd=Twist()
                opening=observed_opening(points,origin,pose,self.c['rack_size_m'][0])
                if rack.get('opening_m') is not None:
                    opening=min(opening,rack['opening_m'])
                if abs(dy)<=.012 and abs(yaw)<=math.radians(2) and swept+.02<=opening:
                    v=self.last['chassis']
                    stable=stable+1 if max(abs(v.vx_mps),abs(v.vy_mps),abs(v.wz_rad_s))<.015 else 0
                    if stable>=3:
                        rack['leg_offsets']=[relative(p,pose) for p in measured_legs]
                        self.standing[code]=measured
                        return dx
                else:
                    stable=0
                    cmd.linear.y=max(-.035,min(.035,.65*dy))
                    cmd.angular.z=max(-.12,min(.12,.7*yaw))
                self.send('/lidar/cmd_vel',cmd)
            raise RackViewLost('雷达未能持续确认四腿与进架净空')
        finally:
            self.send('/lidar/cmd_vel',Twist())
            self.stop_after_motion()

    def verify_lift_position(self,rack):
        base=self.map_base_pose()
        platform=rack_pose(base,self.c['load_center_m'])
        target=self.standing[rack['code']]
        if (math.dist(platform[:2],target[:2])>.025 or
                abs(math.remainder(platform[2]-target[2],math.tau))>math.radians(3)):
            raise RuntimeError('托板位置与雷达架位不符，停止后续动作')

    def move(self,meters):
        self.mode('stop')
        base=self.map_base_pose()
        goal=rack_pose(base,(meters,0.))
        self.report_route('准备',action='进架' if meters>0 else '退架',goal=goal,points=[base[:2],goal[:2]])
        remaining=meters
        while True:
            generation=self.reconnections
            try:
                if self.relative_leg(remaining,generation):return
            except RetreatBlocked as error:
                self.relative_path=None
                if self.relative_id is not None:
                    rid=self.relative_id
                    self.send('/chassis/cancel',UInt32(data=rid))
                    self.mode('stop')
                    self.wait(lambda:generation!=self.reconnections or
                              (self.last['chassis'].action_id==rid and self.last['chassis'].action_state!=ChassisStatus.ACTION_RUNNING),
                              3,'退出货架的定距动作未确认取消')
                else:
                    self.mode('stop')
                self.report_route('退架暂停')
                print(f'退架暂停：{error}，停车后复查剩余退距',flush=True)
                self.pause(.5)
            self.relative_id=self.relative_path=None
            base=self.map_base_pose()
            remaining,lateral=relative(goal,base)
            yaw_error=abs(math.remainder(base[2]-goal[2],math.tau))
            if (abs(lateral)>.015 or yaw_error>math.radians(2) or
                    abs(remaining)>abs(meters)+.015 or remaining*math.copysign(1.,meters)<-.015):
                raise RuntimeError('复位后不在原钻架直线上，保持停车')
            if abs(remaining)<=.01:return
            print(f'恢复定距动作，仅执行剩余 {remaining:.3f} m',flush=True)

    def relative_leg(self,meters,generation):
        odom=self.last['odom'].pose.pose
        q=odom.orientation
        yaw=math.atan2(2*q.w*q.z,1-2*q.z**2)
        self.relative_path=((odom.position.x,odom.position.y),yaw,meters)
        self.check_relative()
        if generation!=self.reconnections:return False
        self.relative_id=int(self.last['chassis'].last_command_id)+1
        rid=self.relative_id
        self.lease='relative'
        self.send('/control/lease',String(data=self.lease))
        timeout_ms=int((abs(meters)/.06+5)*1000)
        self.report_route('已提交')
        self.send('/chassis/move',RelativeMove(request_id=rid,dx_m=meters,max_linear_speed_mps=.06,
                                               max_angular_speed_rad_s=.3,timeout_ms=timeout_ms))
        self.wait(lambda:generation!=self.reconnections or self.last['chassis'].last_command_id==rid,2,'定距请求未确认')
        if generation!=self.reconnections:return False
        if self.last['chassis'].last_command_result!=ChassisStatus.COMMAND_ACCEPTED:raise RuntimeError('定距请求被拒绝')
        self.wait(lambda:generation!=self.reconnections or (self.last['chassis'].action_id==rid and self.last['chassis'].action_state!=ChassisStatus.ACTION_RUNNING),
                  timeout_ms/1000+1,'定距动作超时')
        if generation!=self.reconnections:return False
        if self.last['chassis'].action_state!=ChassisStatus.ACTION_SUCCEEDED:raise RuntimeError('定距动作未完成')
        self.relative_id=self.relative_path=None
        self.mode('stop')
        return True

    def lift(self,up):
        self.mode('stop')
        self.report_route('等待机构',action='举升' if up else '落架')
        self.lift_target=up
        self.recovered_lift=False
        self.phase('lifting' if up else 'lowering')
        angle=self.c['lift_up_deg' if up else 'lift_down_deg']
        self.servo_id+=1
        self.last.pop('servo',None)
        self.send('/servo/command',ServoCommand(request_id=self.servo_id,servo_id=self.c['servo_id'],angle_deg=angle,
                                               speed_deg_s=float(self.c['servo_speed_deg_s'])))
        self.wait(lambda:self.recovered_lift or ('servo' in self.last and self.last['servo'].request_id==self.servo_id),3,'舵机未应答')
        if not self.recovered_lift:
            reply=self.last['servo']
            if not reply.accepted or reply.error_code or reply.servo_id!=self.c['servo_id'] or reply.angle_deg!=angle:
                raise RuntimeError('舵机拒绝或回执不匹配')
            self.phase('lifting' if up else 'lowering',servo_accepted=True,angle_deg=angle)
            delay=self.c.get('up_wait_s' if up else 'down_wait_s')
            if delay is None:
                travel=180. if self.last_lift_angle is None else abs(angle-self.last_lift_angle)
                speed=self.c['servo_speed_deg_s']
                # 180 deg/s² ramp in the bundled ESP32 firmware, plus two PWM ticks.
                delay=travel/speed+speed/180.+.04
            self.pause(delay)
        self.last_lift_angle=angle
        if self.c['lift_policy']=='operator' and not self.recovered_lift:
            token=self.request_load_confirmation(up)
            self.wait(lambda:self.recovered_lift or self.load_confirmed(token),30,'现场确认超时')
        else:print('升降步骤结束',flush=True)
        self.lift_target=None

    def request_load_confirmation(self,up):
        token=secrets.token_hex(4)
        self.last.pop('operator',None)
        self.phase('confirm',token=token,lift_up=up)
        action='货架已离地并仍受托板支撑' if up else '货架已落稳'
        if os.environ.get('DASE_MATCH_TERMINAL')=='1' and sys.stdin.isatty():
            termios.tcflush(sys.stdin.fileno(),termios.TCIFLUSH)
            print(f'确认{action}后，在本终端输入 {token} 并回车',flush=True)
        else:
            print(f'确认{action}后使用网页确认或执行：\n'
                  +'ros2 topic pub --once /match/lift_confirmation std_msgs/msg/String "{data: '+token+'}"',flush=True)
        return token

    def load_confirmed(self,token):
        if os.environ.get('DASE_MATCH_TERMINAL')=='1' and sys.stdin.isatty():
            if not select.select([sys.stdin],[],[],0)[0]:return False
            line=sys.stdin.readline()
            if not line:raise RuntimeError('操作终端已断开')
            if line.strip()==token:return True
            print(f'确认码不匹配，请输入 {token}',flush=True)
            return False
        return 'operator' in self.last and self.last['operator'].data==token

    def footprint(self,loaded):
        r=self.c['radius_loaded_m' if loaded else 'radius_empty_m']/math.cos(math.pi/16)
        msg=Polygon()
        after=time.monotonic()
        outline=None if loaded else self.c.get('empty_footprint_m')
        if outline is None:
            outline=[(r*math.cos(i*math.pi/8),r*math.sin(i*math.pi/8)) for i in range(16)]
        msg.points=[Point32(x=x,y=y) for x,y in outline]
        wanted={}
        for key,pad in (('local_shape',0.),('global_shape',.005)):
            padded=[(x+math.copysign(pad,x),y+math.copysign(pad,y)) for x,y in outline]
            mean_x=sum(x for x,y in padded)/len(padded)
            mean_y=sum(y for x,y in padded)/len(padded)
            wanted[key]=sorted(math.hypot(x-mean_x,y-mean_y) for x,y in padded)
        for _ in range(3):
            for cost in ('local','global'):self.send('/'+cost+'_costmap/footprint',msg)
            self.pause(.1)
        def echoed(key):
            shape=self.last.get(key)
            if shape is None or self.receipt[key]<after:return False
            points=shape.polygon.points
            if len(points)!=len(outline):return False
            cx=sum(p.x for p in points)/len(points)
            cy=sum(p.y for p in points)/len(points)
            actual=sorted(math.hypot(p.x-cx,p.y-cy) for p in points)
            return all(abs(a-b)<=.004 for a,b in zip(actual,wanted[key]))
        self.wait(lambda:echoed('local_shape') and echoed('global_shape'),5,'代价地图未确认装载轮廓')

    def enter_rack(self,rack):
        if self.centerline and not self.direct_test:
            self.face_rack_at_gate(rack)
        retry=False
        while True:
            self.locate_rack(rack,retry)
            try:
                distance=self.align(rack)
                break
            except RackViewLost as error:
                retry=True
                print(f'{error}，停车后换视角重新核码',flush=True)
                self.phase('survey')
                self.pause(1.)
        self.phase('insert')
        self.move(distance)
        self.verify_lift_position(rack)
        return distance

    def nav_recognition_test(self):
        """Drive the empty base and check identities, leaving every rack in place."""
        aisle=None if self.direct_test else self.centerline
        if self.centerline:
            self.leave_start()
        for rack in self.c['racks']:
            self.current_rack=rack['code']
            distance=self.enter_rack(rack)
            print(f"空车已在 {rack['code']} 架下停稳，正在原路倒退 {distance:.3f} m",flush=True)
            self.phase('retreat')
            self.publish_keepout(exiting=(rack,self.standing[rack['code']]))
            self.move(-distance)
            self.publish_keepout()
            self.target_legs=[]
            if aisle:
                gate=aisle.rack_gate(rack['approach'])
                self.follow_centerline((*gate[:2],math.pi))
                self.follow_centerline((aisle.end_x,aisle.y,math.pi))
            if rack['slot'] is not None:
                self.navigate_transit(rack['slot'])
                print('空车到达预设卸货位 '+rack['code'],flush=True)
            if aisle:
                self.follow_centerline((aisle.end_x,aisle.y,math.pi))
        self.current_rack=None
        self.navigate_transit(self.c['end_approach'])
        self.scan('END')
        print('空车测试通过：四架均已核码、钻入并退出，END 已确认',flush=True)

    def race(self):
        for subscription in self.check_only:
            self.destroy_subscription(subscription)
        self.check_only.clear()
        self.running=True
        try:
            self.mode('stop');self.footprint(False);self.scan('START')
            if self.nav_only:
                self.nav_recognition_test()
                return
            self.deadline=time.monotonic()+180
            aisle=self.centerline
            if aisle:
                self.leave_start()
                end_gate=(aisle.end_x,aisle.y,math.pi)
            for r in self.c['racks']:
                self.current_rack=r['code']
                self.phase('approach')
                print('前往 '+r['code'],flush=True)
                self.enter_rack(r)
                self.lift(True)
                self.carrying=True
                self.payload_legs=[world(p,(*self.c['load_center_m'],0.)) for p in r['leg_offsets']]
                self.footprint(True)
                self.standing.pop(r['code'])
                self.publish_keepout()
                self.target_legs=[]
                if aisle:
                    rack_gate=aisle.rack_gate(r['approach'])
                    self.follow_centerline((*rack_gate[:2],math.pi))
                    self.follow_centerline(end_gate)
                self.phase('carry')
                slot=r['slot'] if r['slot'] is not None else self.choose_drop()
                self.navigate_transit(slot)
                self.phase('place')
                while True:
                    deposited=self.measured_drop(slot)
                    base=self.map_base_pose()
                    if self.fixed or r['slot'] is None:
                        grid=self.drop_grid()
                        reserved=self.reserved_goals()
                        try:
                            verify_region_pose(self.c,base,self.other_racks(),grid,reserved)
                        except (ValueError,RuntimeError) as error:
                            print(f'卸货位或退出通道受阻：{error}，保持停车',flush=True)
                            self.pause(.5)
                            continue
                    try:
                        self.check_exit_clearance(base,-self.c['retreat_m'],self.other_racks())
                        break
                    except RetreatBlocked as error:
                        print(f'卸货前等待退出通道：{error}',flush=True)
                        self.pause(.5)
                self.lift(False)
                self.carrying=False
                self.payload_legs=[]
                self.deposited.append(deposited)
                self.phase('retreat')
                self.publish_keepout(exiting=(r,deposited))
                self.move(-self.c['retreat_m'])
                self.publish_keepout()
                self.footprint(False)
                self.completed.append(r['code'])
                if aisle:
                    self.follow_centerline(end_gate)
            self.current_rack=None
            self.phase('finish_scan')
            self.navigate_transit(self.c['end_approach'])
            self.scan('END')
            self.phase('done')
        except BaseException:
            self.running=False
            self.send('/control/mode',String(data='stop'))
            self.send('/chassis/estop',Bool(data=True))
            if self.relative_id:self.send('/chassis/cancel',UInt32(data=self.relative_id))
            if self.nav_goal and self.nav_goal.accepted:self.nav_goal.cancel_goal_async()
            self.send('/lidar/cmd_vel',Twist())
            # Let ROS deliver the stop even when the browser's lease is gone.
            end=time.monotonic()+.3
            while time.monotonic()<end:
                rclpy.spin_once(self,timeout_sec=.03)
            raise
        finally:
            self.running=False
            self.send('/control/mode',String(data='stop'))
            self.report_route('已停止',action='本轮结束')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=['check','run','nav-test'])
    parser.add_argument('config')
    parser.add_argument('--camera-prechecked',action='store_true',help=argparse.SUPPRESS)
    args=parser.parse_args()
    c=load(args.config)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,signal.default_int_handler)
    signal.signal(signal.SIGTERM,signal.default_int_handler)
    if os.environ.get('DASE_MATCH_TERMINAL')=='1':
        signal.signal(signal.SIGHUP,signal.default_int_handler)
    node=Match(c,nav_only=args.action=='nav-test',camera_prechecked=args.camera_prechecked and args.action!='check')
    try:
        node.phase('checking')
        node.wait_links=args.action in ('run','nav-test')
        if c.get('start_region') is not None and not node.gui_id:
            node.localize_start(allow_turn=args.action in ('run','nav-test'))
        node.ready()
        if args.action in ('run','nav-test'):node.race()
    except KeyboardInterrupt:
        node.send('/control/mode',String(data='stop'))
        print('收到中止信号，已请求停车，结束本轮。',flush=True)
        raise SystemExit(130)
    except Exception as exc:
        node.phase('failed', reason=str(exc))
        if node.gui_id:
            end=time.monotonic()+.35
            while time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.05)
        raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
