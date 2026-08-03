#!/usr/bin/env python3
# Pose keeper: makes AMCL localize itself across navigation restarts.
#
# While navigation runs, it saves every /amcl_pose update to a small JSON file
# (throttled to 1 Hz). On startup, if that file exists, it republishes the
# saved pose to /initialpose until AMCL confirms with an /amcl_pose message,
# so nobody has to click "set robot pose" after every relaunch.
#
# The saved pose is only as good as where the robot physically is: if the
# robot was carried somewhere else while navigation was down, override it
# from the map viewer as before (a manual /initialpose always wins — the
# keeper immediately saves whatever AMCL converges to).

import json
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

DEFAULT_POSE_FILE = os.path.expanduser('~/.ros/linorobot2_last_pose.json')
# Covariance used when restoring: loose enough that AMCL can pull the estimate
# onto the scan, tight enough that it does not need a global search.
RESTORE_COV_XY = 0.25       # m^2  (0.5 m std dev)
RESTORE_COV_YAW = 0.14      # rad^2 (~21 deg std dev)


class PoseKeeper(Node):
    def __init__(self):
        super().__init__('pose_keeper')
        self.declare_parameter('pose_file', DEFAULT_POSE_FILE)
        self.declare_parameter('restore', True)
        self.pose_file = self.get_parameter('pose_file').value

        self.localized = False
        self.last_save = self.get_clock().now()

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 1)
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl_pose, 10)

        self.restore_pose = None
        if self.get_parameter('restore').value:
            self.restore_pose = self.load_saved_pose()
        if self.restore_pose is not None:
            x, y, yaw = self.restore_pose
            self.get_logger().info(
                f'restoring saved pose x={x:.2f} y={y:.2f} yaw={yaw:.2f} '
                f'from {self.pose_file}')
            # Republish every 2 s until AMCL answers with an /amcl_pose,
            # give up after 60 s (AMCL not up / user localized manually).
            self.attempts = 0
            self.restore_timer = self.create_timer(2.0, self.try_restore)
        else:
            self.get_logger().info(
                f'no saved pose at {self.pose_file}; waiting for a manual '
                'initial pose, will save from then on')

    def load_saved_pose(self):
        try:
            with open(self.pose_file) as f:
                d = json.load(f)
            return float(d['x']), float(d['y']), float(d['yaw'])
        except FileNotFoundError:
            return None
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().warn(f'ignoring corrupt pose file: {e}')
            return None

    def try_restore(self):
        if self.localized or self.attempts >= 30:
            self.restore_timer.cancel()
            return
        self.attempts += 1
        x, y, yaw = self.restore_pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp.sec = 0  # stamp 0 = "latest", avoids TF extrapolation
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = RESTORE_COV_XY
        cov[35] = RESTORE_COV_YAW
        msg.pose.covariance = cov
        self.pub.publish(msg)

    def on_amcl_pose(self, msg):
        self.localized = True
        now = self.get_clock().now()
        if (now - self.last_save).nanoseconds < 1e9:
            return
        self.last_save = now
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        data = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': yaw,
        }
        tmp = self.pose_file + '.tmp'
        try:
            os.makedirs(os.path.dirname(self.pose_file), exist_ok=True)
            with open(tmp, 'w') as f:
                json.dump(data, f)
            os.replace(tmp, self.pose_file)  # atomic: no torn file on power cut
        except OSError as e:
            self.get_logger().warn(f'could not save pose: {e}')


def main():
    rclpy.init()
    node = PoseKeeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
