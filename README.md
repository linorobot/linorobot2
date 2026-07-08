**Documentation: [https://linorobot.github.io/linorobot2](https://linorobot.github.io/linorobot2)**

---

<!-- Build Status populated by Github Actions runs -->
ROS 2 Distro | Branch | Build status
:----------: | :----: | :----------:
**Jazzy** | [`jazzy`](../../tree/jazzy) | [![Jazzy Build](../../actions/workflows/build.yml/badge.svg?branch=jazzy)](../../actions/workflows/build.yml?branch=jazzy)

# linorobot2

![linorobot2](docs/assets/linorobot2.gif)

linorobot2 is a ROS2 package that takes your robot from bare hardware to fully autonomous navigation. Whether you're building a physical robot from accessible parts, simulating in Gazebo, learning Nav2, or prototyping new hardware, linorobot2 gives you a complete, working foundation with Nav2, SLAM Toolbox, and robot_localization already wired together.

Supported base configurations: **2WD**, **4WD**, and **Mecanum drive**.

## What Can You Do With It?

- **Build a real autonomous robot.** Follow the [hardware guide](https://github.com/linorobot/linorobot2_hardware) to assemble your robot from off-the-shelf parts, flash the micro-ROS firmware, and run SLAM and Nav2 with a single command.
- **Simulate in Gazebo.** A pre-configured robot URDF with lidar, depth camera, and IMU is ready to spawn. The same launch files and Nav2 configuration work for both physical and simulated robots, with no separate config to maintain.
- **Simulate your real environment.** Convert a floor plan image or a SLAM-generated map directly into a Gazebo world. Test your ROS2 application in the exact same layout as your physical space, with the same obstacles your lidar sees, with no need to run the robot.
- **Learn Nav2.** The documentation walks through the [Nav2 setup guides](https://docs.nav2.org/setup_guides/index.html) journey step by step: base controller, odometry, sensors, transforms, SLAM, and navigation. Each concept is explained before it is configured.
- **Prototype new hardware.** Use the templated URDF as a starting point for your own robot design. Swap in your CAD meshes, adjust the sensor poses, and validate the kinematics in Gazebo before cutting any parts.
- **Build ROS2 applications.** The simulation stack provides a consistent, reproducible environment for developing and testing autonomy code including path planners, state machines, and perception pipelines, without needing physical hardware on hand.

## Features

### Nav2, SLAM Toolbox, and robot_localization (pre-integrated)

linorobot2 ships with working configurations for the full ROS2 autonomous navigation stack. Nav2, SLAM Toolbox, and the robot_localization EKF are configured and ready to go. The same YAML files are used by both the physical robot and the Gazebo simulation, so tuning in simulation transfers directly to hardware.

![Architecture](docs/assets/linorobot2_launchfiles.png)

### Pre-configured robot with sensors

The robot URDF is templated with a 2D lidar, an RGBD depth camera, and an IMU already included and positioned. Changing the robot's dimensions or sensor mounting positions is a matter of editing one properties file. The URDF is also a solid starting point for building a more detailed model: add your CAD meshes and the rest of the stack continues to work.

### Simulate your real environment

Two tools in `linorobot2_gazebo` let you bring your physical environment into Gazebo:

- **`image_to_gazebo`**: a GUI tool that takes any floor plan image (PNG, JPG, BMP, etc.), lets you calibrate its real-world scale and set the coordinate origin interactively, then generates a complete Gazebo world: 3D wall mesh, model SDF, and world SDF.
- **`create_worlds_from_maps`**: a batch CLI tool that converts all SLAM maps in `linorobot2_navigation/maps/` into Gazebo worlds in one command.

Both tools produce a Gazebo world that matches the geometry your lidar sees in the real environment. You can develop and test your Nav2 application in simulation with full confidence that the obstacle layout is accurate, then deploy to the physical robot without surprises.

### Wide sensor support

linorobot2 supports a broad range of 2D lidars and RGBD depth cameras out of the box. The install script sets up the correct driver and topic remappings automatically. For a full list, see the [Sensors](docs/05_sensors.md) documentation.

**Selected supported lidars:** RPLIDAR A1/A2/A3/S1/S2/S3/C1, LD06, LD19, STL27L, YDLIDAR, XV11, Intel RealSense (as lidar), ZED (as lidar)

**Supported depth cameras:** Intel RealSense D435/D435i, ZED/ZED2/ZED2i/ZED Mini, OAK-D/OAK-D Lite/OAK-D Pro

### Build your own robot

Detailed hardware documentation covering motor driver configuration and micro-ROS firmware for Teensy and compatible boards is at [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware). The firmware publishes odometry and IMU data over micro-ROS so the microcontroller integrates seamlessly as a ROS2 node.

![Microcontroller architecture](docs/assets/microcontroller_architecture.png)

## Quickstart

All commands below run on the robot computer unless noted. SLAM and navigation launch files are identical for physical and simulated robots.

### Physical Robot

This is the exact terminal-by-terminal sequence for **this robot** (Jetson +
Teensy 4.1 base over micro-ROS, MPU6050 IMU direct on the Jetson's I2C bus 7,
RPLIDAR A3). **Every terminal must source ROS first:**

```bash
source /opt/ros/jazzy/setup.bash
source ~/linorobot2_ws/install/setup.bash
```

#### Quick start — one command, 2 terminals (recommended)

The whole ROS 2 mapping stack now starts from a single launch file, so you only
need **two terminals** — the map is a browser tab, not a third terminal.

**Terminal 1 — the entire robot stack.** micro-ROS agent (so teleop drives the
motors), RPLIDAR A3 cropped to the **front 180°** (the rear is dropped so the
battery behind the robot never appears as a phantom obstacle), MPU6050 IMU, rf2o
laser odometry, SLAM Toolbox, and the web map viewer. One `Ctrl-C` stops all of it:
```bash
ros2 launch linorobot2_bringup robot.launch.py
```

**Terminal 2 — drive to map** (ramped keyboard teleop; needs its own terminal
because it captures keystrokes):
```bash
cd ~/Desktop/linorobot2 && ./teleop_keyboard.py
```

**See the map — from your laptop's browser, no third terminal.** The launch
serves the live map on **port 8000**. Open `http://<robot-ip>:8000` on the same
LAN, or tunnel it over SSH:
```bash
ssh -L 8000:localhost:8000 jetson1@<robot-ip>   # then open http://localhost:8000
```
See [Watching the map over SSH](#watching-the-map-over-ssh) for details.

Handy arguments:
```bash
ros2 launch linorobot2_bringup robot.launch.py micro_ros:=false    # you push the robot by hand
ros2 launch linorobot2_bringup robot.launch.py map_viewer:=false   # don't serve port 8000
ros2 launch linorobot2_bringup robot.launch.py rviz:=true          # local screen / ssh -X
ros2 launch linorobot2_bringup robot.launch.py lidar_port:=/dev/ttyUSB0 base_serial_port:=/dev/ttyACM0
```

Save the map when it looks complete (while SLAM is still running — the map only
lives in slam_toolbox's memory until saved):
```bash
cd ~/Desktop/linorobot2/linorobot2_navigation/maps && ros2 run nav2_map_server map_saver_cli -f my_new_map --ros-args -p save_map_timeout:=10000.0
```
To use it for navigation, set `MAP_NAME` in
`linorobot2_navigation/launch/navigation.launch.py` to the new name.

> This uses lidar + IMU SLAM (rf2o laser odometry), which needs no wheel
> encoders. The step-by-step sequence below is the fuller base + EKF workflow;
> use it when you want each piece in its own terminal or need to troubleshoot.

#### Autonomous navigation — 3 commands

Drive the robot to goals on a saved map (AMCL localization + Nav2). Speed is
capped at **0.05 m/s** in `linorobot2_navigation/config/navigation.yaml`
(`desired_linear_vel` + the `velocity_smoother` limits — change both to go faster).

**Terminal 1 — robot base, no SLAM** (AMCL owns localization in this mode;
never run SLAM and navigation together):
```bash
ros2 launch linorobot2_bringup robot.launch.py slam:=false
```

**Terminal 2 — Nav2 with the saved map** (loads the `MAP_NAME` default from
`navigation.launch.py`; pass `map:=/abs/path/to/map.yaml` to override):
```bash
ros2 launch linorobot2_navigation navigation.launch.py
```

**Browser — tell AMCL where the robot is** (do this within ~30 s of Terminal 2
starting, or the Nav2 lifecycle nodes time out waiting for the map frame). Open
`http://<robot-ip>:8000`, press **📍 set robot pose**, click where the robot is,
then click a point it is facing. The red footprint snaps into place when AMCL
localizes.

**Terminal 3 — send a goal.** Click the destination on the map page and paste
the command it generates:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.20, y: 0.40}, orientation: {w: 1.0}}}}"
```
The command streams feedback and returns on arrival; a new goal preempts the
old one. Hard stop: `Ctrl-C` the goal, or run teleop and hit space.

If a goal is rejected ("Action server is inactive"), the pose was set too late —
reactivate the stranded Nav2 nodes:
```bash
for n in planner_server behavior_server velocity_smoother collision_monitor bt_navigator waypoint_follower route_server docking_server; do ros2 lifecycle set /$n activate; done
```

#### Step-by-step (manual) sequence

Start the terminals in order (1 → 6).

**Terminal 1 — Boot the base (micro-ROS agent + base node):**
```bash
ros2 launch linorobot2_bringup bringup.launch.py
```
Wait for the micro-ROS agent to print `session established` before continuing.

**Terminal 2 — IMU (MPU6050 on I2C bus 7 @ 0x68):**
```bash
ros2 launch mpu6050_imu imu.launch.py i2c_bus:=7 i2c_addr:=104
```
The IMU is wired directly to the Jetson, **not** through the Teensy, so it is a
separate launch. Note `i2c_addr:=104` (decimal 0x68) — **no trailing period**,
or it fails with `invalid literal for int()`.

**Terminal 3 — SLAM (also starts the RPLIDAR driver and the robot_localization EKF):**
```bash
ros2 launch linorobot2_navigation slam.launch.py
```

**Terminal 4 — Watch the map over SSH (headless, no RViz):**
```bash
python3 ~/Desktop/map_viewer.py
```
Then browse to `http://<robot-ip>:8000` from any machine on the LAN (or tunnel
with `ssh -L 8000:localhost:8000 <user>@<robot-ip>` and open
`http://localhost:8000`). The page renders `/map` with the robot pose and
auto-refreshes; it shows "waiting for /map" until SLAM is publishing. See
[Watching the map over SSH](#watching-the-map-over-ssh) below for details.

**Terminal 5 — Drive to map the area (ramped keyboard teleop):**
```bash
cd ~/Desktop/linorobot2 && ./teleop_keyboard.py
```
Keys: `i`/`k` forward/back, `j`/`l` rotate left/right, `u`/`o`/`m`/`.` drive +
turn, any other key stops, `Ctrl-C` quits. `q/z` scale all speeds, `w/x` linear
only, `e/c` angular only. This robot's `+linear.x` drives it physically
backward, so the script flips the sign internally — `i` really is forward.

This teleop is **acceleration-ramped**: keypresses set a *target* velocity and a
50 Hz loop eases the published `/cmd_vel` toward it at a capped rate (default
`0.05 m/s²` linear, `0.1 rad/s²` angular), so the drivetrain never sees an
instant jump — it ramps smoothly through zero on a forward↔reverse reversal and
ramps down to a stop. This protects the gears from shock loads. Override the
ramp without editing the file:
```bash
./teleop_keyboard.py --ros-args -p linear_accel:=0.1 -p angular_accel:=0.5
```

> **Note:** an earlier `drive_telem.py` existed because the *original*
> `teleop_keyboard.py` published only **one** `/cmd_vel` per keypress, which the
> firmware's 200 ms cmd_vel failsafe would zero out between taps. The ramped
> `teleop_keyboard.py` above now publishes continuously at 50 Hz, so it sustains
> motion and satisfies the failsafe on its own — use it instead.

**Terminal 6 — Log IMU signals to CSV (optional):**
```bash
ros2 run mpu6050_imu imu_logger
```
Subscribes to `/imu/data` and, per recording session, writes **two separate
CSVs** to `~/imu_logs/`:
1. `imu_z_displacement_<timestamp>.csv` — the z (up) acceleration,
   gravity-corrected and double-integrated into vertical displacement
   (`t_s, dt_s, az_raw, az_dynamic, vel_z, disp_z`).
2. `imu_xz_accel_<timestamp>.csv` — the magnitude of the x+z (forward+up)
   acceleration vector (`t_s, dt_s, ax_dynamic, az_dynamic, accel_xz_magnitude`).

Press **Enter** to start/stop recording — each ON→OFF cycle produces a new,
timestamped CSV pair. `q`+Enter quits. Gravity/bias is estimated while idle, so
keep the robot still for ~1 s before the first record. Override the output
location with `--ros-args -p output_dir:=<dir> -p file_prefix:=<name>`.
> Note: double-integrated MEMS displacement drifts over time; keep runs short
> for meaningful `disp_z`. Velocity and displacement reset to zero each session.

**Save the map** (when the map looks complete):
```bash
cd ~/Desktop/linorobot2/linorobot2_navigation/maps
ros2 run nav2_map_server map_saver_cli -f <map_name> --ros-args -p save_map_timeout:=10000.
```

**Navigate autonomously** (replace Terminal 3's SLAM):
```bash
ros2 launch linorobot2_navigation navigation.launch.py map:=<path_to_map>/<map_name>.yaml
```

### Watching the map over SSH

Over a plain SSH session there is no display, so instead of RViz this robot
serves the live map as a web page from `~/Desktop/map_viewer.py`. The quick-start
`robot.launch.py` starts it automatically (disable with `map_viewer:=false`); in
the step-by-step sequence it is Terminal 4. It subscribes to `/map` and the robot
pose and serves an auto-refreshing PNG on port 8000 — no ROS or RViz needed on
the viewing machine.

```bash
# On the robot (Terminal 4):
python3 ~/Desktop/map_viewer.py

# From your laptop, either browse directly (same LAN):
#   http://<robot-ip>:8000
# ...or tunnel the port over SSH and use localhost:
ssh -L 8000:localhost:8000 <user>@<robot-ip>
#   then open http://localhost:8000
```

If you'd rather use RViz on your **laptop** (both machines on the same LAN and
same `ROS_DOMAIN_ID`, default `0`; `/map` and `/scan` are discovered over DDS):

```bash
# On the laptop (one-time): copy the saved view config from the robot
scp jetson1@<robot-ip>:~/Desktop/linorobot2/slam_imu.rviz .

# On the laptop, each time you want to watch:
source /opt/ros/jazzy/setup.bash
rviz2 -d slam_imu.rviz
```

### Simulated Robot

**Terminal 1:Start Gazebo:**
```bash
ros2 launch linorobot2_gazebo gazebo.launch.py
```

**Terminal 2:Run SLAM or navigation** (same commands as physical robot, add `sim:=true`):
```bash
ros2 launch linorobot2_navigation slam.launch.py sim:=true
# or
ros2 launch linorobot2_navigation navigation.launch.py map:=<path_to_map>/<map_name>.yaml sim:=true
```

## Tools

### image_to_gazebo

Convert any floor plan or building layout image into a Gazebo world with a GUI:

```bash
ros2 run linorobot2_gazebo image_to_gazebo
```

Load your image, calibrate the scale by clicking two known points, set the coordinate origin, and click Generate. The tool writes the STL mesh, model SDF, and world SDF to the package's `models/` and `worlds/` directories. Launch the generated world with:

```bash
ros2 launch linorobot2_gazebo gazebo.launch.py world_name:=<world_name>
```

### create_worlds_from_maps

Batch-convert all saved SLAM maps to Gazebo worlds in one command:

```bash
ros2 run linorobot2_gazebo create_worlds_from_maps
```

This reads every YAML file in `linorobot2_navigation/maps/`, extrudes the occupancy grid into a 3D wall mesh, and writes a Gazebo world for each map. Useful for keeping simulation worlds in sync after a mapping session.

## Documentation

Full documentation covering installation, base controller, odometry, sensors, transforms, mapping, navigation and more is in the [linorobot2 documentation website](https://linorobot.github.io/linorobot2/)

### Documentation Maintenance

Documentation is stored in the [`docs/`](docs/) directory and its subdirectories as a set of markdown files which are published to
the linorobot2 documentation website (implemented using GitHub Pages).
This happens automatically on every push to the current actively maintained main branch (e.g. `jazzy`).

You can browse the docs locally and review your updates prior to committing and publishing:

```bash
sudo apt install mkdocs-material
mkdocs serve
```

Then open [http://127.0.0.1:8000](http://127.0.0.1:8000) in your browser.


## Installation

See the Installation page of the [linorobot2 documentation website](https://linorobot.github.io/linorobot2/) for full installation instructions covering the Workstation, Robot Computer,
Microcontroller, and Docker.

## Useful Resources

- [Nav2 Setup Guides](https://docs.nav2.org/setup_guides/index.html)
- [Gazebo ROS2 Overview](https://gazebosim.org/docs/latest/ros2_overview)
- [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware)
- [linorobot2_viz](https://github.com/linorobot/linorobot2_viz)
