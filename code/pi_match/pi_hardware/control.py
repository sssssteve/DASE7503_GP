"""Hardware gate with an expiring mission lease. No queued motion on owner changes."""
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String,Bool
from adapter import Adapter


class HardwareGate(Adapter):
    def __init__(self):
        super().__init__()
        self.deadline = 0.
        self.action_lease = False
        self.estop = self.create_publisher(Bool,'/chassis/estop',10)
        self.create_subscription(String,'/control/lease',self.renew,10)
        self.create_subscription(Twist,'/lidar/cmd_vel',lambda msg:self.receive('lidar',msg),10)

    def select(self,msg):
        if msg.data=='lidar':
            self.mode='lidar'
            self.inputs.clear()
            self.command.publish(Twist())
        elif msg.data!='vision':
            super().select(msg)

    def renew(self, msg):
        self.deadline = time.monotonic()+.6
        self.action_lease = msg.data=='relative'

    def tick(self):
        if time.monotonic()>self.deadline:
            self.mode='stop'
            self.inputs.clear()
            if self.action_lease:
                self.estop.publish(Bool(data=True))
                self.action_lease=False
        super().tick()


if __name__=='__main__':
    rclpy.init()
    n=HardwareGate()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():n.command.publish(Twist())
        n.destroy_node()
        rclpy.try_shutdown()
