"""ROS interface adapter and exclusive velocity selection; no hardware transport."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from chassis_interfaces.msg import ChassisStatus
from tf2_ros import TransformBroadcaster

class Adapter(Node):
    def __init__(self):
        super().__init__('chassis_odom_and_control_gate')
        self.mode='stop';self.inputs={};self.status_time=0.;self.fault=True
        self.odom=self.create_publisher(Odometry,'/odom',10);self.tf=TransformBroadcaster(self)
        self.command=self.create_publisher(Twist,'/cmd_vel',10)
        self.report=self.create_publisher(String,'/control/status',10)
        self.create_subscription(ChassisStatus,'/chassis/status',self.feedback,10)
        self.create_subscription(String,'/control/mode',self.select,10)
        self.create_subscription(Twist,'/nav_cmd_vel',lambda m:self.receive('nav',m),10)
        self.create_subscription(Twist,'/vision/cmd_vel',lambda m:self.receive('vision',m),10)
        self.create_subscription(Twist,'/mapping/cmd_vel',lambda m:self.receive('mapping',m),10)
        self.create_timer(.05,self.tick)

    def select(self,msg):
        if msg.data in ['stop','nav','vision','mapping']:
            self.mode=msg.data;self.inputs.clear();self.command.publish(Twist())

    def receive(self,name,msg):self.inputs[name]=(time.monotonic(),msg)

    def feedback(self,msg):
        self.status_time=time.monotonic();self.fault=bool(msg.fault or msg.emergency_stopped)
        values=[msg.x_m,msg.y_m,msg.yaw_rad,msg.vx_mps,msg.vy_mps,msg.wz_rad_s]
        if not all(math.isfinite(v) for v in values):self.fault=True;return
        m=Odometry();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='odom';m.child_frame_id='base_link'
        m.pose.pose.position.x=msg.x_m;m.pose.pose.position.y=msg.y_m
        m.pose.pose.orientation.z=math.sin(msg.yaw_rad/2);m.pose.pose.orientation.w=math.cos(msg.yaw_rad/2)
        m.twist.twist.linear.x=float(msg.vx_mps);m.twist.twist.linear.y=float(msg.vy_mps);m.twist.twist.angular.z=float(msg.wz_rad_s)
        # Provisional covariance, not a fitted real-wheel uncertainty model.
        for i in [0,7,35]:m.pose.covariance[i]=.0025;m.twist.covariance[i]=.0025
        for i in [14,21,28]:m.pose.covariance[i]=1e6;m.twist.covariance[i]=1e6
        self.odom.publish(m)
        t=TransformStamped();t.header=m.header;t.child_frame_id=m.child_frame_id
        t.transform.translation.x=msg.x_m;t.transform.translation.y=msg.y_m;t.transform.rotation=m.pose.pose.orientation
        self.tf.sendTransform(t)

    def tick(self):
        now=time.monotonic();m=Twist();selected='stop'
        if not self.fault and now-self.status_time<.4 and self.mode in self.inputs:
            stamp,candidate=self.inputs[self.mode]
            if now-stamp<.25 and all(math.isfinite(v) for v in [candidate.linear.x,candidate.linear.y,candidate.angular.z]):
                m=candidate;selected=self.mode
        speed=math.hypot(m.linear.x,m.linear.y)
        if speed>.25:m.linear.x*=.25/speed;m.linear.y*=.25/speed
        m.angular.z=max(-1.,min(1.,m.angular.z))
        self.command.publish(m)
        self.report.publish(String(data=json.dumps({'mode':self.mode,'selected':selected,
            'cmd_publishers':self.count_publishers('/cmd_vel'),'velocity':[m.linear.x,m.linear.y,m.angular.z]})))

if __name__=='__main__':
    rclpy.init();n=Adapter()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
