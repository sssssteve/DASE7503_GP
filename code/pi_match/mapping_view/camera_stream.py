"""Sample the newest raw image at 5 Hz on an independent preview connection."""
import base64
import json

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image


def main():
    rclpy.init()
    node=rclpy.create_node('local_camera_preview')
    bridge=CvBridge()
    cv2.setNumThreads(1)
    latest=None
    last_stamp=None

    def receive(msg):
        nonlocal latest
        latest=msg

    def emit():
        nonlocal last_stamp
        if latest is None:return
        stamp=Time.from_msg(latest.header.stamp)
        age=(node.get_clock().now()-stamp).nanoseconds/1e9
        if stamp.nanoseconds==last_stamp or not -.05<=age<1:return
        last_stamp=stamp.nanoseconds
        try:
            pixels=bridge.imgmsg_to_cv2(latest,'bgr8')
            h,w=pixels.shape[:2]
            if w>480:
                pixels=cv2.resize(pixels,(480,max(1,round(h*480/w))),interpolation=cv2.INTER_AREA)
            ok,jpeg=cv2.imencode('.jpg',pixels,[cv2.IMWRITE_JPEG_QUALITY,58])
        except (CvBridgeError,ValueError,cv2.error):
            return
        if ok:
            print(json.dumps(dict(stamp=str(last_stamp),jpeg=base64.b64encode(jpeg).decode('ascii'))),flush=True)

    node.create_subscription(Image,'/camera/image_raw',receive,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_timer(.2,emit)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,BrokenPipeError):pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
