#!/usr/bin/env python3
# Checks which way the lidar is pointing relative to the robot.
#
# Hold your hand ~30 cm from the lidar, directly ahead of the robot's nose.
# The script prints the angle of the nearest object in the scan:
#     ~0 deg   -> lidar zero is aligned with robot front (correct)
#     ~180 deg -> lidar is mounted backwards
#     ~+90 deg -> lidar zero points to the robot's left
#     ~-90 deg -> lidar zero points to the robot's right
#
# Usage (on the Jetson, with the lidar driver running):
#     python3 lidar_alignment_check.py [scan_topic]
# Default topic is /scan_raw (the unfiltered 360 deg scan). Use /scan if
# you are running the driver without the angle filter.

import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def sector(deg):
    if -45 <= deg <= 45:
        return 'FRONT'
    if 45 < deg <= 135:
        return 'LEFT'
    if -135 <= deg < -45:
        return 'RIGHT'
    return 'BACK'


class LidarAlignmentCheck(Node):
    def __init__(self, topic):
        super().__init__('lidar_alignment_check')
        self.create_subscription(LaserScan, topic, self.on_scan, 10)
        self.get_logger().info(f'Listening on {topic} — hold your hand ~30 cm from the lidar')

    def on_scan(self, msg):
        nearest_range = float('inf')
        nearest_angle = None
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < nearest_range:
                nearest_range = r
                nearest_angle = msg.angle_min + i * msg.angle_increment

        if nearest_angle is None:
            print('no valid returns in scan')
            return

        deg = math.degrees(nearest_angle)
        fov = math.degrees(msg.angle_max - msg.angle_min)
        print(f'nearest object: {nearest_range:.2f} m at {deg:+6.1f} deg '
              f'({sector(deg)})   [scan FOV: {fov:.0f} deg]')


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/scan_raw'
    rclpy.init()
    node = LidarAlignmentCheck(topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
