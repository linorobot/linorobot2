#!/usr/bin/env python3
# Customized keyboard teleop for linorobot2.
#
# Differences from the stock ros2 teleop_twist_keyboard:
#   * i = forward, k = backward (the natural i/k axis). The stock node leaves
#     k unbound (so it did nothing) and put reverse on the comma key.
#   * Forward/back signs are inverted relative to stock because this robot's
#     +linear.x drives it *backward* physically. Here i sends -x so the robot
#     actually moves forward, and k sends +x to move backward.
#   * Default linear speed starts at 0.05 m/s.
#   * ACCELERATION RAMPING: keypresses set a *target* velocity; a fixed-rate
#     loop eases the published /cmd_vel toward that target at a capped rate so
#     the drivetrain never sees an instant jump (protects the gears). This also
#     ramps smoothly through zero on a forward<->reverse reversal or a stop.
#
# Run with:  ./teleop_keyboard.py     (or: python3 teleop_keyboard.py)
# Publishes geometry_msgs/Twist on /cmd_vel.

import sys
import threading

import geometry_msgs.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
linorobot2 keyboard teleop (ramped)
-----------------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

   i : forward          k : backward
   j : rotate left      l : rotate right
   u/o : forward + turn     m/. : backward + turn

anything else : stop (ramps down to zero)

q/z : increase/decrease all speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit (ramps to a stop first)
"""

# (x, y, z, th). x is the linear "forward intent" -- it is multiplied by -1
# below before publishing so i ends up moving the robot physically forward.
moveBindings = {
    'i': (1, 0, 0, 0),    # forward
    'k': (-1, 0, 0, 0),   # backward
    'j': (0, 0, 0, 1),    # rotate left
    'l': (0, 0, 0, -1),   # rotate right
    'u': (1, 0, 0, 1),    # forward + left
    'o': (1, 0, 0, -1),   # forward + right
    'm': (-1, 0, 0, 1),   # backward + left
    '.': (-1, 0, 0, -1),  # backward + right
    ',': (-1, 0, 0, 0),   # backward (alias for k)
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

# This robot's +linear.x drives it physically backward, so flip the sign once
# here. Pressing i -> intent +1 -> published linear.x = -speed -> moves forward.
LINEAR_SIGN = -1

# --- ramping defaults (overridable via ROS params) --------------------------
# Max rate of change of the published command. Lower = gentler on the gears.
PUBLISH_RATE = 50.0     # Hz -- how often /cmd_vel is (re)published
LINEAR_ACCEL = 0.05     # m/s^2   -- linear velocity slew limit
ANGULAR_ACCEL = 0.1     # rad/s^2 -- angular velocity slew limit


def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %.2f\tturn %.2f ' % (speed, turn)


def approach(current, target, max_step):
    """Move `current` toward `target` by at most `max_step`."""
    if target > current:
        return min(current + max_step, target)
    return max(current - max_step, target)


def main():
    settings = saveTerminalSettings()

    rclpy.init()
    node = rclpy.create_node('teleop_keyboard')

    speed = node.declare_parameter('speed', 0.05).value
    turn = node.declare_parameter('turn', 1.0).value
    publish_rate = node.declare_parameter('publish_rate', PUBLISH_RATE).value
    linear_accel = node.declare_parameter('linear_accel', LINEAR_ACCEL).value
    angular_accel = node.declare_parameter('angular_accel', ANGULAR_ACCEL).value

    pub = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)

    dt = 1.0 / publish_rate
    lin_step = linear_accel * dt      # max linear change per tick
    ang_step = angular_accel * dt     # max angular change per tick

    # Shared state between the key thread (writes targets) and the ramp timer
    # (reads targets, updates current, publishes). Guarded by `lock`.
    lock = threading.Lock()
    state = {
        'target_lin': 0.0, 'target_ang': 0.0,   # where we want to be
        'cur_lin': 0.0, 'cur_ang': 0.0,          # where we currently are
    }

    def ramp_and_publish():
        with lock:
            tl, ta = state['target_lin'], state['target_ang']
            cl = approach(state['cur_lin'], tl, lin_step)
            ca = approach(state['cur_ang'], ta, ang_step)
            state['cur_lin'], state['cur_ang'] = cl, ca

        twist = geometry_msgs.msg.Twist()
        twist.linear.x = cl
        twist.angular.z = ca
        pub.publish(twist)

    node.create_timer(dt, ramp_and_publish)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    x = 0.0
    th = 0.0
    status = 0.0

    try:
        print(msg)
        print(vels(speed, turn))
        print('ramp: %.2f m/s^2 lin, %.2f rad/s^2 ang @ %.0f Hz'
              % (linear_accel, angular_accel, publish_rate))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                th = 0.0
                if key == '\x03':
                    break

            # Update the *target*; the ramp timer eases the actual command to it.
            with lock:
                state['target_lin'] = x * speed * LINEAR_SIGN
                state['target_ang'] = th * turn

    except Exception as e:
        print(e)

    finally:
        # Ramp down to a stop instead of slamming zero, then shut down.
        with lock:
            state['target_lin'] = 0.0
            state['target_ang'] = 0.0
        stopper = geometry_msgs.msg.Twist()
        # Publish a decaying command until we've reached zero (bounded loop).
        max_ticks = int(publish_rate * 5) + 1
        for _ in range(max_ticks):
            with lock:
                cl = approach(state['cur_lin'], 0.0, lin_step)
                ca = approach(state['cur_ang'], 0.0, ang_step)
                state['cur_lin'], state['cur_ang'] = cl, ca
            stopper.linear.x = cl
            stopper.angular.z = ca
            pub.publish(stopper)
            if cl == 0.0 and ca == 0.0:
                break
            threading.Event().wait(dt)

        rclpy.shutdown()
        spinner.join()
        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
