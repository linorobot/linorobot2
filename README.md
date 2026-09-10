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

**Terminal 1:Boot the robot:**
```bash
ros2 launch linorobot2_bringup bringup.launch.py
```
Wait for the micro-ROS agent to print `session established` before continuing.

**Terminal 2:Create a map:**
```bash
ros2 launch linorobot2_navigation slam.launch.py
```

**Terminal 3:Drive to map the area:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Save the map:**
```bash
cd linorobot2/linorobot2_navigation/maps
ros2 run nav2_map_server map_saver_cli -f <map_name> --ros-args -p save_map_timeout:=10000.
```

**Terminal 2:Navigate autonomously:**
```bash
ros2 launch linorobot2_navigation navigation.launch.py map:=<path_to_map>/<map_name>.yaml
```

Visualize from your host machine at any point:
```bash
ros2 launch linorobot2_viz slam.launch.py        # during mapping
ros2 launch linorobot2_viz navigation.launch.py  # during navigation
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

### Linorobot2 Console

A browser-based cockpit for the robot computer. Monitor robot health, manage micro-ROS agent connections, tune Nav2 & SLAM parameters visually, trigger one-click bringup/SLAM/Nav2, and drive via on-screen virtual gamepad:

```bash
python3 tools/console/web/server.py --port 8090
```

Open [http://localhost:8090](http://localhost:8090) in your browser. All console launchers and parameters are self-contained in `tools/console/`. See [`tools/console/README.md`](tools/console/README.md) for details.

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
