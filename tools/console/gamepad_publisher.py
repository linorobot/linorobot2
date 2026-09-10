#!/usr/bin/env python3
"""Persistent cmd_vel publisher for the Console's virtual gamepad.

The browser gamepad produces a stream of small velocity updates as the stick is
dragged. Spawning `ros2 topic pub` per update is not usable -- node startup and
discovery cost far more than the interval between updates -- so Console starts
this once and feeds it target velocities on stdin, one "lx ly az" line each.

It republishes the latest target at a fixed rate, which is what a real teleop
node does: the base stops when cmd_vel goes quiet, so the command has to be
held. It also carries a deadman timeout -- if the browser stops sending (tab
closed, laptop asleep, Wi-Fi dropped) the robot is commanded to zero rather
than driving on the last command it heard.
"""

import argparse
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class GamepadPublisher(Node):
    def __init__(self, topic, rate_hz, timeout_s):
        super().__init__("console_gamepad")
        self.pub = self.create_publisher(Twist, topic, 10)
        self.timeout_s = timeout_s
        self.lock = threading.Lock()
        self.target = (0.0, 0.0, 0.0)
        self.last_cmd_time = 0.0
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            "publishing %s at %g Hz, deadman timeout %gs" % (topic, rate_hz, timeout_s)
        )

    def tick(self):
        with self.lock:
            lx, ly, az = self.target
            stale = (time.monotonic() - self.last_cmd_time) > self.timeout_s
        if stale:
            lx = ly = az = 0.0
        msg = Twist()
        msg.linear.x = lx
        msg.linear.y = ly
        msg.angular.z = az
        self.pub.publish(msg)

    def set_target(self, lx, ly, az):
        with self.lock:
            self.target = (lx, ly, az)
            self.last_cmd_time = time.monotonic()

    def stop(self):
        with self.lock:
            self.target = (0.0, 0.0, 0.0)


def read_stdin(node):
    for line in sys.stdin:
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "stop":
            node.stop()
            continue
        try:
            lx, ly, az = (float(p) for p in parts[:3])
        except ValueError:
            continue
        node.set_target(lx, ly, az)
    # stdin closed: the browser is gone, so stop the robot
    node.stop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/cmd_vel")
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--timeout", type=float, default=0.5)
    args = ap.parse_args()

    rclpy.init()
    node = GamepadPublisher(args.topic, args.rate, args.timeout)
    threading.Thread(target=read_stdin, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # leave the base stopped, not coasting on the last command
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
