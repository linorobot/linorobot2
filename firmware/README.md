# linorobot2 Teensy firmware — AK10-9 (MIT mode)

micro-ROS firmware that turns a **Teensy 4.1** into a linorobot2 base
controller for two **CubeMars AK10-9** motors driven in **MIT mode over CAN2**,
with an optional **MPU6050** IMU.

It replaces the bench-test sketch (`teensy41_can1_can3_test.ino`): same MIT CAN
command encoding, but instead of serial keystrokes it now:

| Topic | Direction | Type | Purpose |
|-------|-----------|------|---------|
| `cmd_vel` | subscribe | `geometry_msgs/Twist` | velocity commands (Nav2 / teleop) |
| `odom/unfiltered` | publish | `nav_msgs/Odometry` | wheel odometry from motor feedback (for SLAM) |
| `imu/data` | publish | `sensor_msgs/Imu` | raw IMU (only if an MPU6050 is wired) |

These are exactly the topics linorobot2's `bringup.launch.py` expects, so once
flashed the Teensy joins the ROS 2 graph automatically.

## Wiring (unchanged from the bench test)

- **CAN2**: pin 0 = CRX2, pin 1 = CTX2 → 3.3 V CAN transceiver → motors (1 Mbps).
- **Motors**: left = controller ID `0x68` (cmd `0x868`), right = `0x69` (cmd `0x869`).
- **MPU6050** (optional): SDA = pin 18, SCL = pin 19, 3.3 V.

## Configure before flashing — `include/config.h`

1. `WHEEL_DIAMETER` and `LR_WHEELS_DISTANCE` — **measure these on your robot.**
   They set the odometry scale; wrong values = drifting SLAM maps.
2. `LEFT_MOTOR_DIR` / `RIGHT_MOTOR_DIR` — flip a sign if a wheel spins the wrong
   way (mirrored drivetrains usually have one side negated; defaults match the
   bench test: left `+1`, right `-1`).
3. `SPIN_KP / SPIN_KD / SPIN_TORQUE_FF` — copied from the bench test. If
   low-speed motion is jerky during SLAM, lower `SPIN_TORQUE_FF`.

## Build & flash

Requires [PlatformIO](https://platformio.org/) (`pip install platformio`).

```bash
cd firmware
pio run -e teensy41 -t upload
```

The first build downloads micro-ROS for the `jazzy` distro
(`board_microros_distro` in `platformio.ini`) — **this must match the ROS 2
distro on your robot computer.**

## Run it with linorobot2

On the robot computer:

```bash
# Terminal 1 — starts the micro-ROS agent and bridges the Teensy
ros2 launch linorobot2_bringup bringup.launch.py
# wait for: session established

# Terminal 2 — drive it
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3 — map
ros2 launch linorobot2_navigation slam.launch.py
```

The onboard LED is **on** when connected to the agent, **off** otherwise. The
motors are actively held at zero whenever the agent is disconnected or `cmd_vel`
goes silent for `CMD_VEL_TIMEOUT_MS`.

## Verify odometry (do this before trusting SLAM)

```bash
ros2 topic echo /odom/unfiltered
```

- **No messages / all zeros while wheels turn** → the firmware isn't decoding
  motor feedback. See "MIT feedback" below.
- **Scale check**: push (or drive) the robot exactly 1 m forward and confirm
  `pose.pose.position.x` ≈ 1.0. If it reads e.g. 0.5 or 2.0, fix
  `WHEEL_DIAMETER`. Spin in place 360° and confirm yaw returns to start; if not,
  fix `LR_WHEELS_DISTANCE`.

## MIT-mode notes & caveats

- **Velocity is at the output shaft (rad/s).** Unlike servo/ERPM mode, MIT mode
  needs no gear-ratio or pole-pair conversion — wheel angular velocity maps 1:1
  to the MIT `v_des` and feedback velocity. (Assumes the wheel sits directly on
  the output shaft.)
- **Feedback decoding** (`ak10_mit.h::handleFrame`) assumes the standard MIT
  reply frame: `[id, pos(16), vel(12), cur(12)]`, routed by controller id in
  byte 0. Some CubeMars firmware versions differ slightly. If `/odom/unfiltered`
  stays zero, flash the original bench-test sketch, watch the `RX ...` serial
  dump while a wheel turns, and adjust the byte layout in `handleFrame` to match.
- **Enter motor mode**: the bench test (and this firmware by default) does not
  send the MIT "enter motor mode" frame. If your motors ignore commands after
  flashing, uncomment the `ak10EnterMotorMode(...)` calls in `setup()`.
- This firmware does **not** depend on the linorobot2 ROS 2 packages at build
  time; it only shares their topic contract. It lives in this repo for
  convenience and version-tracking alongside the rest of your robot.
