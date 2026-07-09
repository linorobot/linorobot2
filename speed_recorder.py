#!/usr/bin/env python3
# Records how fast the robot is actually moving and turning, to CSV.
#
# Subscribes to every velocity source the robot publishes and samples them all
# into one row at 20 Hz, so commanded vs measured motion can be compared:
#     /odom_rf2o      nav_msgs/Odometry   laser scan-matching odometry (measured)
#     odom/unfiltered nav_msgs/Odometry   motor-side odometry from the Teensy
#     /imu/data       sensor_msgs/Imu     gyro z — the best turning-rate measurement
#     /cmd_vel        geometry_msgs/Twist commanded velocity (for comparison)
#
# Usage (on the robot, with robot.launch.py running; drive with teleop in
# another terminal):
#     python3 speed_recorder.py [--dir ~/speed_logs]
#
# Press Enter to START recording, Enter again to STOP — each ON->OFF cycle
# writes a new  speed_log_<timestamp>.csv  (same session UX as imu_logger).
# q+Enter quits. A live speed/turn readout prints while the node runs.
#
# CSV columns: t_s, dt_s, rf2o_vx, rf2o_wz, wheel_vx, wheel_wz, imu_wz,
# cmd_vx, cmd_wz.  A cell is left empty when that source has not published
# for >1 s, so dropouts show up as gaps instead of silently repeating stale
# values. Staleness is judged by receive time on this machine, not header
# stamps — the Teensy's micro-ROS clock is not synced to the Jetson's.

import argparse
import csv
import math
import os
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

SAMPLE_HZ = 20.0
STALE_S = 1.0  # a source older than this is logged as empty


class SpeedRecorder(Node):
    def __init__(self, out_dir):
        super().__init__('speed_recorder')
        self.out_dir = out_dir

        # source name -> (values tuple, receive time.monotonic())
        self.latest = {}
        self.lock = threading.Lock()
        self.rec_lock = threading.Lock()

        # micro-ROS (odom/unfiltered) and the IMU publish best-effort; rf2o and
        # teleop publish reliable. A best-effort subscription matches both, so
        # use sensor QoS everywhere rather than guessing per topic.
        qos = qos_profile_sensor_data
        self.create_subscription(Odometry, '/odom_rf2o', self.on_rf2o, qos)
        self.create_subscription(Odometry, 'odom/unfiltered', self.on_wheel, qos)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, qos)

        self.recording = False
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.t0 = None
        self.t_prev = None
        self.rows = 0

        self.create_timer(1.0 / SAMPLE_HZ, self.on_sample)

    def store(self, name, values):
        with self.lock:
            self.latest[name] = (values, time.monotonic())

    def on_rf2o(self, msg):
        self.store('rf2o', (msg.twist.twist.linear.x, msg.twist.twist.angular.z))

    def on_wheel(self, msg):
        self.store('wheel', (msg.twist.twist.linear.x, msg.twist.twist.angular.z))

    def on_imu(self, msg):
        self.store('imu', (msg.angular_velocity.z,))

    def on_cmd(self, msg):
        self.store('cmd', (msg.linear.x, msg.angular.z))

    def fresh(self, name, now):
        with self.lock:
            entry = self.latest.get(name)
        if entry is None or now - entry[1] > STALE_S:
            return None
        return entry[0]

    def on_sample(self):
        now = time.monotonic()
        rf2o = self.fresh('rf2o', now)
        wheel = self.fresh('wheel', now)
        imu = self.fresh('imu', now)
        cmd = self.fresh('cmd', now)

        # rec_lock: the sampler runs in the spin thread while Enter toggles
        # recording from the stdin thread — never write to a file mid-close.
        with self.rec_lock:
            if self.recording:
                if self.t0 is None:
                    self.t0 = now
                    self.t_prev = now
                row = [f'{now - self.t0:.6f}', f'{now - self.t_prev:.6f}']
                row += self.cells(rf2o, 2) + self.cells(wheel, 2) \
                    + self.cells(imu, 1) + self.cells(cmd, 2)
                self.csv_writer.writerow(row)
                self.t_prev = now
                self.rows += 1

        # Live readout: prefer measured (rf2o speed, gyro turn), fall back to
        # motor odometry when a source is out.
        speed = rf2o[0] if rf2o else (wheel[0] if wheel else None)
        turn = imu[0] if imu else (rf2o[1] if rf2o else (wheel[1] if wheel else None))
        speed_s = f'{speed:+6.3f} m/s' if speed is not None else '  --   '
        turn_s = (f'{turn:+6.3f} rad/s ({math.degrees(turn):+7.1f} deg/s)'
                  if turn is not None else '  --   ')
        rec_s = f'REC {self.rows:5d} rows' if self.recording else 'idle (Enter=rec)'
        print(f'\r  speed {speed_s}   turn {turn_s}   [{rec_s}] ',
              end='', flush=True)

    @staticmethod
    def cells(values, n):
        if values is None:
            return [''] * n
        return [f'{v:.6f}' for v in values]

    def start_recording(self):
        os.makedirs(self.out_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.out_dir, f'speed_log_{stamp}.csv')
        n = 1
        while os.path.exists(path):  # two sessions within the same second
            path = os.path.join(self.out_dir, f'speed_log_{stamp}_{n}.csv')
            n += 1
        with self.rec_lock:
            self.csv_path = path
            self.csv_file = open(path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['t_s', 'dt_s', 'rf2o_vx', 'rf2o_wz',
                                      'wheel_vx', 'wheel_wz', 'imu_wz',
                                      'cmd_vx', 'cmd_wz'])
            self.t0 = None
            self.t_prev = None
            self.rows = 0
            self.recording = True
        print(f'\nrecording -> {path}')

    def stop_recording(self):
        with self.rec_lock:
            self.recording = False
            self.csv_file.close()
            self.csv_file = None
        print(f'\nsaved {self.rows} rows -> {self.csv_path}')

    def toggle_recording(self):
        if self.recording:
            self.stop_recording()
        else:
            self.start_recording()


def key_loop(node):
    # Runs in its own thread: Enter toggles recording, q+Enter quits.
    for line in sys.stdin:
        if line.strip().lower() == 'q':
            break
        node.toggle_recording()
    rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='Record robot speed and turn rate to CSV.')
    parser.add_argument('--dir', default=os.path.expanduser('~/speed_logs'),
                        help='output directory (default: ~/speed_logs)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = SpeedRecorder(args.dir)
    print('speed_recorder: Enter = start/stop recording, q+Enter = quit')

    threading.Thread(target=key_loop, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.recording:
            node.stop_recording()
        rclpy.try_shutdown()
        print()


if __name__ == '__main__':
    main()
