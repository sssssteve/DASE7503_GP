import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from dase7503_slam.configuration import load_interfaces


class InputCheck(Node):
    def __init__(self, config):
        super().__init__("dase7503_input_check")
        self.config = config
        self.scan = None
        self.odom = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(LaserScan, config["scan_topic"], self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, config["odom_topic"], self.on_odom,
                                 qos_profile_sensor_data)

    def on_scan(self, message):
        self.scan = message

    def on_odom(self, message):
        self.odom = message

    def problem(self):
        config = self.config
        if self.scan is None or self.odom is None:
            return "Waiting for live LaserScan and Odometry messages"
        if self.scan.header.frame_id != config["lidar_frame"]:
            return "LaserScan frame_id does not match lidar_frame"
        if (self.odom.header.frame_id != config["odom_frame"] or
                self.odom.child_frame_id != config["base_frame"]):
            return "Odometry frames do not match odom_frame/base_frame"
        for message in (self.scan, self.odom):
            stamp = Time.from_msg(message.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if stamp.nanoseconds == 0 or not 0 <= age <= 2.0:
                return "Zero, stale (>2s), or future message timestamp; check clocks"
        if self.scan.angle_increment <= 0 or not any(
            math.isfinite(value) and self.scan.range_min <= value <= self.scan.range_max
            for value in self.scan.ranges
        ):
            return "LaserScan has no usable ranges or invalid angle_increment"
        stamp = Time.from_msg(self.scan.header.stamp)
        try:
            for parent, child in ((config["odom_frame"], config["base_frame"]),
                                  (config["base_frame"], config["lidar_frame"])):
                self.buffer.lookup_transform(parent, child, stamp, timeout=Duration())
        except TransformException as error:
            return "TF unavailable at scan timestamp: " + str(error)
        return None


def main():
    parser = argparse.ArgumentParser(description="Check actual lidar, odometry and TF inputs")
    parser.add_argument("--config", required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be a positive finite number")
    try:
        config = load_interfaces(args.config)
    except (ValueError, OSError) as error:
        parser.exit(2, str(error) + "\n")
    rclpy.init()
    node = InputCheck(config)
    problem = "No inputs received"
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            problem = node.problem()
            if problem is None:
                print("PASS: live scan, odometry and TF at scan time. This is not a SLAM accuracy test.")
                return 0
        print("FAIL: " + str(problem))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
