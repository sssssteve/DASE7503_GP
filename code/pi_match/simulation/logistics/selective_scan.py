"""Reuse the team scanner and correlate each competition scan with its request."""
import json

import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from scan_node import QrScanNode
from target_reader import TargetReader


class MatchScanner(QrScanNode):
    def __init__(self):
        super().__init__()
        self.reader=TargetReader()
        self.request_id=None
        self.create_subscription(String,'/vision/expected',self.expected,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String,'/match/scan_target',self.request,10)

    def expected(self,msg):
        if self.request_id is None:
            self.reader.expected=msg.data

    def request(self,msg):
        try:
            request=json.loads(msg.data)
            token,target=request['id'],request['expected']
        except (ValueError,TypeError,KeyError):
            return
        if self.image_sub is None and isinstance(token,str) and isinstance(target,str):
            self.request_id=token
            self.reader.expected=target
            normal_timeout=self.timeout
            try:
                if request.get('quick_rack_scan') is True:
                    self.timeout=min(3.,normal_timeout)
                self.start_scan(None)
            finally:
                self.timeout=normal_timeout

    def status(self,state,**details):
        super().status(state,request_id=self.request_id,expected=self.reader.expected,**details)

    def stop_scan(self):
        super().stop_scan()
        self.request_id=None


if __name__=='__main__':
    rclpy.init();node=MatchScanner()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
