#!/usr/bin/env python3
# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CommandTimeout(Node):
    def __init__(self):
        super().__init__('command_timeout')
        self.prev_cmd_time_ = self.get_clock().now()
        self.zero_cmd_sent_ = True
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        brake_timer = self.create_timer(0.2, self.brake_timer_callback)

        twist_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)
        twist_subscription

    def brake_timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.prev_cmd_time_).nanoseconds

        #5Hz
        if dt >= 2e+8 and not self.zero_cmd_sent_:
            self.zero_cmd_sent_ = True
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.angular.z = 0.0
            self.twist_publisher_.publish(twist_msg)

    def twist_callback(self, msg):
        if msg.linear.x == 0 and msg.linear.y == 0 and msg.angular.z == 0:
            return

        self.zero_cmd_sent_ = False
        self.prev_cmd_time_ = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)

    command_timeout = CommandTimeout()
    rclpy.spin(command_timeout)
    command_timeout.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()