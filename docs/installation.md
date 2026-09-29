# Installation

This guide walks you through installing linorobot2, whether you're setting up a Physical Robot or running a Simulated Robot in the Gazebo simulator on your desktop
or in the cloud.

There are two repos you will need to clone and install:

1. [linorobot2](https://github.com/linorobot/linorobot2). This repo is needed
for all supported configurations and contains the linorobot2-specific part
of Robot Software.
2. [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware). This
repo is only needed for Physical Robot configurations, and contains Robot Firmware.

The following steps walk you through the installation of each.

## Physical Robot

You need to install Robot Software on the Workstation when you are building a
Physical Robot because the build of Robot Firmware depends on it. It is also
needed to run the Robot Wifi Configuration.

If you are building the Onboard Robot Computer configuration, you also need
to install Robot Software on the onboard Robot Computer.

See [here](../system_configurations/) for definitions of the configurations.

### Prerequisites

The Workstation and Robot Computer need supported versions of Ubuntu
and ROS2. See the System Requirements section of the main page
for supported versions. If you haven't installed it yet,
use the [ros2me](https://github.com/linorobot/ros2me) installer script,
which has been tested on x86 and ARM boards including Raspberry Pi 4
and Nvidia Jetson series.

### Workstation and Robot Computer Installation

The easiest way to get Robot Software installed on the robot computer
is to run the provided bash script from the root of this repository. It
installs all dependencies, sets the required environment variables,
and creates a `linorobot2_ws` in your `$HOME` directory.

The *colcon build* step should complete without errors.

```bash
source /opt/ros/<ros_distro>/setup.bash
cd /tmp
wget https://raw.githubusercontent.com/linorobot/linorobot2/${ROS_DISTRO}/install.bash
bash install.bash --base <robot_type> [--laser <laser_sensor>] [--depth <depth_sensor>] [--workspace <path>]
source ~/.bashrc
cd ~/linorobot2_ws
colcon build
```

Passing `--base` runs the full installation: workspace setup, sensor drivers, micro-ROS, and the linorobot2 package, and exports the required env variables to `~/.bashrc`. Omitting `--base` installs only the specified sensor drivers, which is useful when integrating into an existing workspace.

**robot_type:** choose the base that matches your hardware:

| Value | Description |
|-------|-------------|
| `2wd` | 2 wheel differential drive robot |
| `4wd` | 4 wheel or tracked skid-steer robot |
| `mecanum` | Mecanum drive robot |

**laser_sensor:** choose your 2D lidar or depth-camera-as-lidar:

| Value | Sensor |
|-------|--------|
| `a1` | [RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1) |
| `a2` | [RPLIDAR A2](https://www.slamtec.ai/product/slamtec-rplidar-a2/) |
| `a3` | [RPLIDAR A3](https://www.slamtec.ai/product/slamtec-rplidar-a3/) |
| `s1` | [RPLIDAR S1](https://www.slamtec.com/en/Lidar/S1) |
| `s2` | [RPLIDAR S2](https://www.slamtec.com/en/Lidar/S2) |
| `s3` | [RPLIDAR S3](https://www.slamtec.com/en/Lidar/S3) |
| `c1` | [RPLIDAR C1](https://www.slamtec.ai/product/slamtec-rplidar-a3/) |
| `ld06` | [LD06 LIDAR](https://www.ldrobot.com/ProductDetails?sensor_name=STL-06P) |
| `ld19` | [LD19/LD300 LIDAR](https://www.ldrobot.com/ProductDetails?sensor_name=STL-19P) |
| `stl27l` | [STL27L LIDAR](https://www.ldrobot.com/ProductDetails?sensor_name=STL-27L) |
| `ydlidar` | [YDLIDAR](https://www.ydlidar.com/lidars.html) |
| `xv11` | [XV11](http://xv11hacking.rohbotics.com/mainSpace/home.html) |
| `realsense` * | [Intel RealSense](https://www.intelrealsense.com/stereo-depth/) D435, D435i |
| `zed` * | [Zed](https://www.stereolabs.com/zed) |
| `zed2` * | [Zed 2](https://www.stereolabs.com/zed-2) |
| `zed2i` * | [Zed 2i](https://www.stereolabs.com/zed-2i) |
| `zedm` * | [Zed Mini](https://www.stereolabs.com/zed-mini) |

Sensors marked with `*` are depth cameras. When used as a laser sensor, the launch files will run [depthimage_to_laserscan](https://github.com/ros-perception/depthimage_to_laserscan) to convert the depth image to a laser scan. Omit `--laser` if you have no laser sensor.

**depth_sensor:** choose your RGBD camera (optional, separate from the laser sensor):

| Value | Sensor |
|-------|--------|
| `realsense` | [Intel RealSense](https://www.intelrealsense.com/stereo-depth/) D435, D435i |
| `zed` | [Zed](https://www.stereolabs.com/zed) |
| `zed2` | [Zed 2](https://www.stereolabs.com/zed-2) |
| `zed2i` | [Zed 2i](https://www.stereolabs.com/zed-2i) |
| `zedm` | [Zed Mini](https://www.stereolabs.com/zed-mini) |
| `oakd` | [OAK-D](https://shop.luxonis.com/collections/oak-cameras-1/products/oak-d) |
| `oakdlite` | [OAK-D Lite](https://shop.luxonis.com/collections/oak-cameras-1/products/oak-d-lite-1) |
| `oakdpro` | [OAK-D Pro](https://shop.luxonis.com/collections/oak-cameras-1/products/oak-d-pro) |

### Workstation: RVIZ (Remote Visualization) Support

When working with a Physical Robot, you'll want to visualize what's
happening from the Workstation (the laser scan, the map being built,
and the robot's pose) without running a simulation. Install
[linorobot2_viz](https://github.com/linorobot/linorobot2_viz) on your
host machine for this purpose:

```bash
cd <host_machine_ws>
git clone https://github.com/linorobot/linorobot2_viz src/linorobot2_viz
rosdep update && rosdep install --from-path src --ignore-src -y
colcon build
source install/setup.bash
```

This package is kept separate from the main installation to keep the Robot Computer lean. Both machines must be on the same ROS2 network (same `ROS_DOMAIN_ID`).

### Workstation: Install Robot Firmware and Build System

Robot Firmware is build with the PlatformIO build system. First, you install
PlatformIO, then you clone and build Robot Firmware.

#### Install PlatformIO

Download and install [Platformio](https://platformio.org/) - it
allows you to develop, configure, and upload Robot Firmware to the
Microcontroller without the Arduino IDE. This means that you can upload
the firmware using a USB serial cable, or even remotely using OTA (Over The
Air update).

```bash
    
    curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
    python3 get-platformio.py
```

Add PlatformIO to your $PATH:

```bash
    echo "PATH=\"\$PATH:\$HOME/.platformio/penv/bin\"" >> $HOME/.bashrc
    source $HOME/.bashrc
```

#### Download and build Robot Firmware

Clone Robot Firmware from the linorobot2_hardware repo and build it. The
build should complete without errors.

```bash
    cd $HOME
    git clone https://github.com/linorobot/linorobot2_hardware
    cd linorobot2_hardware/firmware
    pio run -e esp32
```

#### (Deprecated) Teensy UDEV Rule

Download the udev rules from Teensy's website:

    wget https://www.pjrc.com/teensy/00-teensy.rules

and copy the file to /etc/udev/rules.d :

    sudo cp 00-teensy.rules /etc/udev/rules.d/

---

## Simulated Robot

The simulation runs on your Workstation or in the cloud and does not
require a Physical Robot at all. It's a great way to tune Nav2 parameters
and test your configuration before deploying to hardware. If you only
intend to run simulations, and not build a Physical Robot, you can use
the following simplified Robot Software installation procedure.

### 2.1 Install Robot Software Packages

```bash
cd <host_machine_ws>
git clone -b $ROS_DISTRO https://github.com/linorobot/linorobot2 src/linorobot2
git clone https://github.com/linorobot/linorobot2_viz src/linorobot2_viz
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent --skip-keys micro_ros_agent
colcon build
source install/setup.bash
```

> The `--skip-keys` flags prevent a known [rosdep issue](https://github.com/micro-ROS/micro_ros_setup/issues/138) with micro-ROS keys. Always include them when running `rosdep install` on this workspace.

#### Define Robot Type

Set the `LINOROBOT2_BASE` environment variable to match the robot base you want to simulate:

```bash
echo "export LINOROBOT2_BASE=2wd" >> ~/.bashrc
source ~/.bashrc
```

Available values: `2wd`, `4wd`, `mecanum`.

The simulation package already includes RVIZ configurations, so you do not need to install `linorobot2_viz` separately when using the simulation.

### 2.2 Test simulation using default robot and world

Start gazebo with the default 2wd robot in the default turtlebot3 world. The robot must be spawned
offset from the origin, where a piller is located.

```bash
ros2 launch linorobot2_gazebo gazebo.launch.py spawn_x:=0.5
```

In another window, launch navigation with default parameters for the default robot:

```bash
ros2 launch linorobot2_navigation navigation.launch.py sim:=true
```

In a third window, launch rviz:

```bash
ros2 launch linorobot2_viz navigation.launch.py sim:=true
```

Set the initial pose to just in front of the center pillar. Set a target pose and watch it navigate to the
target pose.

## Docker Option

If you're not running a supported combination of Ubuntu and ROS2 versions,
or if you're running simulation in the cloud where you don't even have Ubuntu,
Docker is a convenient alternative for running simulation.

Install Docker using the [official instructions](https://docs.docker.com/engine/install/) and follow the [post-install steps](https://docs.docker.com/engine/install/linux-postinstall/) to run Docker without `sudo`.

Customize `docker/.env` if needed (e.g., to change the robot type), then build the image:

```bash
git clone https://github.com/linorobot/linorobot2.git
cd linorobot2/docker
docker compose build
```

To start the simulation in Docker:

```bash
# Start Gazebo
docker compose up gazebo

# In another terminal, start navigation
export SIM=true
docker compose up navigate

# In another terminal, open RViz
docker compose up rviz-nav
```

If you see "Unable to create rendering window" errors with Gazebo, run `xhost +` first.

You can also run everything in daemon mode:

```bash
export SIM=true
export RVIZ=true
docker compose up -d gazebo navigate
```

Shut everything down with `docker compose down`. View logs with `docker compose logs`.

> Note: you cannot mix Docker containers with native ROS2 nodes on the same machine.
