"""Read-only status display for the empty-base rack test."""
import math
import time
from pathlib import Path

import rclpy
from chassis_interfaces.msg import ChassisStatus
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, LaserScan


def main():
    rclpy.init()
    node=rclpy.create_node('rack_test_monitor')
    topics={'/scan':LaserScan, '/scan_body_filtered':LaserScan, '/chassis/status':ChassisStatus,
            '/odom':Odometry, '/camera/camera_info':CameraInfo}
    counts=dict.fromkeys(topics,0)
    last={}
    details={}
    previous=time.monotonic()

    def receive(topic,msg):
        counts[topic]+=1
        last[topic]=time.monotonic()
        if topic=='/chassis/status':
            details[topic]=f'fault={msg.fault} 急停={int(msg.emergency_stopped)}'
        elif topic=='/camera/camera_info':
            details[topic]=f'{msg.width}×{msg.height}'
        elif topic=='/scan_body_filtered':
            details[topic]=f'{sum(math.isfinite(r) and msg.range_min<=r<msg.range_max for r in msg.ranges)} 点'

    def report():
        nonlocal previous
        now=time.monotonic()
        values=[]
        for topic in topics:
            age=now-last[topic] if topic in last else None
            if age is None:
                value='等待首帧'
            elif age>1:
                value=f'{age:.1f}s 无新消息'
            else:
                value=f'{counts[topic]/(now-previous):.1f}Hz '+details.get(topic,'')
            values.append(f'{topic} {value.strip()}')
            counts[topic]=0
        previous=now
        devices=sorted(p.resolve().name for p in Path('/dev/serial/by-id').glob('usb-1a86_USB_Single_Serial_*-if00') if p.exists())
        print(time.strftime('%H:%M:%S')+'  雷达 USB '+(','.join(devices) or '未枚举')+' | '+' | '.join(values),flush=True)

    for topic,kind in topics.items():
        node.create_subscription(kind,topic,lambda msg,t=topic:receive(t,msg),qos_profile_sensor_data)
    node.create_timer(3,report)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        if rclpy.ok():raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
