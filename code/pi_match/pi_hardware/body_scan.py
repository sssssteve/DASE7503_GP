"""Publish the measured scan without body hits and unsupported single-beam returns."""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from config import load
from start_region import filter_scan


def main():
    config=load(sys.argv[1])
    rclpy.init(args=[])
    node=Node('body_scan_filter')
    output=node.create_publisher(LaserScan,'/scan_body_filtered',qos_profile_sensor_data)

    def receive(msg):
        if msg.header.frame_id=='laser_frame':
            filter_scan(msg,config['laser_mount'],config['base_size_m'],config.get('scan_mask_box_m'),
                        config.get('scan_neighbor_gap_m'),config.get('scan_echo_windows',[]),config.get('scan_body_padding_m',0.),config.get('body_center_x_m',0.))
            output.publish(msg)

    node.create_subscription(LaserScan,'/scan',receive,qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
