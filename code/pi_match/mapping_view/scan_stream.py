"""Read-only ROS scan stream for the SSH monitor. No command publishers."""
from collections import deque
import hashlib
import json
import math
import os
import sys
import time

from chassis_interfaces.msg import ChassisStatus
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


def main():
    rclpy.init()
    node=Node('local_web_scan_monitor')
    tf=Buffer()
    listener=TransformListener(tf,node)
    root=Path(os.environ.get('DASE_PI_MATCH_ROOT','~/DASE7503_Pi5')).expanduser()
    hardware=root/'pi_hardware'
    sys.path.insert(0,str(hardware))
    from start_region import inside_body, scan_components
    config=yaml.safe_load((hardware/'match.yaml').read_text())
    layout=[dict(name='P'+str(i),center=r['rack_hint'][:2],size=config['rack_size_m'],yaw=r['rack_hint'][2])
            for i,r in enumerate(config['racks'],1)]
    center_x=config.get('body_center_x_m',0.)
    outlines=[]
    mx,my,_,_,_,yaw=config['laser_mount']
    for low,high,rmin,rmax in config.get('scan_echo_windows',[]):
        outlines.append([[mx+r*math.cos(math.radians(a)+yaw),my+r*math.sin(math.radians(a)+yaw)]
                        for a,r in ((low,rmin),(low,rmax),(high,rmax),(high,rmin))])
    latest=None
    grid=None
    received={}
    base_status=''
    localization=''
    route=None
    trail=deque(maxlen=150)
    last_pose_stamp=None
    def on_route(msg):
        nonlocal route
        try:route=json.loads(msg.data)
        except ValueError:route=None

    def on_chassis(msg):
        nonlocal base_status
        received['chassis']=time.monotonic()
        base_status=f'fault={msg.fault}，急停={int(msg.emergency_stopped)}'
    def on_localization(msg):
        nonlocal localization
        localization=msg.data
    def on_scan(msg):
        nonlocal latest
        latest=msg
        received['scan']=time.monotonic()
    def on_map(msg):
        nonlocal grid
        info=msg.info
        grid=(hashlib.sha256(msg.data.tobytes()).hexdigest() if msg.header.frame_id=='map'
              and (info.width,info.height)==(155,104) and abs(info.resolution-.02)<1e-5
              and abs(info.origin.position.x+.04)<1e-5 and abs(info.origin.position.y+.04)<1e-5
              and abs(info.origin.orientation.w)>0.999999 else None)
    def emit():
        nonlocal last_pose_stamp,route
        now=time.monotonic()
        result={'raw':[], 'filtered':[], 'pose':None, 'points':[], 'status':'等待 /scan','monitor':True,'rack_layout':layout,'mask_outlines':outlines}
        if latest is not None:
            msg=latest
            stamp=Time.from_msg(msg.header.stamp)
            age=(node.get_clock().now()-stamp).nanoseconds/1e9
            if -.05<=age<1 and msg.header.frame_id=='laser_frame':
                box=config.get('scan_mask_box_m')
                _,xs,ys,masked,isolated=scan_components(msg,config['laser_mount'],config['base_size_m'],box,
                                                    config.get('scan_neighbor_gap_m'),config.get('scan_echo_windows',[]),config.get('scan_body_padding_m',0.),center_x)
                result['raw']=[[round(x,3),round(y,3)] for x,y in zip(xs.tolist(),ys.tolist())]
                outside=~(masked|isolated)
                result.update(mask_box=box,body_size=config['base_size_m'],body_center_x=center_x,isolated_hits=int(isolated.sum()),
                              body_padding=config.get('scan_body_padding_m',0.),
                              cluster_hits=int((masked & ~inside_body(xs,ys,config['base_size_m'],config.get('scan_body_padding_m',0.),center_x)).sum()))
                external=list(zip(xs[outside].tolist(),ys[outside].tolist()))
                result['filtered']=[[round(x,3),round(y,3)] for x,y in external]
                result['body_hits']=len(xs)-len(external)
                result.update(status='雷达在线 · 等待地图定位',scan_stamp=stamp.nanoseconds)
                if grid:
                    try:
                        b=tf.lookup_transform('map','base_link',stamp).transform
                        q=b.rotation
                        h=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                        bx,by=b.translation.x,b.translation.y
                        result.update(pose=[bx,by,h],grid_hash=grid,status='雷达在线 · 地图定位可用',
                                      points=[[round(bx+math.cos(h)*u-math.sin(h)*v,3),
                                               round(by+math.sin(h)*u+math.cos(h)*v,3)] for u,v in external])
                    except TransformException:
                        pass
            else:
                result['status']='扫描过期或坐标系不匹配'
        if not node.count_publishers('/match/route_preview'):
            route=None
            trail.clear()
        if route and grid:
            result['route']=route
            result['route_grid_hash']=grid
            pose=result['pose']
            if pose and result['scan_stamp']!=last_pose_stamp:
                if not trail or math.dist(trail[-1][1],pose[:2])>.008:
                    trail.append((now,pose[:2]))
                last_pose_stamp=result['scan_stamp']
            while trail and now-trail[0][0]>30:trail.popleft()
            result['pose_trail']=[point for _,point in trail]
        result['sources']={k:round(now-v,1) for k,v in received.items()}
        result['base_status']=base_status
        result['localization']=localization if localization and node.count_publishers('/match/localization_status') else ''
        print(json.dumps(result,separators=(',',':')),flush=True)
    node.create_subscription(LaserScan,'/scan',on_scan,qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid,'/map',on_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(ChassisStatus,'/chassis/status',on_chassis,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/odom',lambda _:received.update(odom=time.monotonic()),qos_profile_sensor_data)
    node.create_subscription(String,'/match/route_preview',on_route,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String,'/match/localization_status',on_localization,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_timer(.2,emit)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,BrokenPipeError):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
