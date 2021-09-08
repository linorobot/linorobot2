## Manual installation of linorobot2 package on robot computer

### 1. Install micro-ROS and its dependencies

#### 1.1 Source your ROS2 distro and workspace
If it's your first time using ROS2 and haven't created your ROS2 workspace yet, you can check out [ROS2 Creating a Workspace](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html) tutorial.

    source /opt/ros/<your_ros_distro>/setup.bash
    cd <your_ws>
    colcon build
    source install/setup.bash

#### 1.2 Download and install micro-ROS:

    cd <your_ws>
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup src/micro_ros_setup
    sudo apt install python3-vcstool
    sudo apt update && rosdep update
    rosdep install --from-path src --ignore-src -y
    colcon build
    source install/setup.bash

#### 1.3 Setup micro-ROS agent:

    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    source install/setup.bash

* You can ignore `1 package had stderr output: microxrcedds_agent` after building your workspace. 

#### 1.4 Install LIDAR ROS2 drivers:
RPLIDAR:

    sudo apt install ros-$ROS_DISTRO-rplidar-ros
    cd /tmp
    wget https://raw.githubusercontent.com/allenh1/rplidar_ros/ros2/scripts/rplidar.rules
    sudo cp rplidar.rules /etc/udev/rules.d/
    sudo service udev reload
    sudo service udev restart

LDLIDAR:

    cd <your_ws>
    git clone https://github.com/linorobot/ldlidar src/ldlidar

#### 1.5 Install depth sensor drivers:
Intel RealSense:

    sudo apt install ros-$ROS_DISTRO-realsense2-camera

### 2. Download linorobot2 and its dependencies:

#### 2.1 Download linorobot2:

    cd <your_ws> 
    git clone https://github.com/linorobot/linorobot2 src/linorobot2

#### 2.2 Ignore Gazebo Packages on robot computer (optional)

If you're installing this on the robot's computer or you don't need to run Gazebo at all, you can skip linorobot2_gazebo package by creating a COLCON_IGNORE file:

    cd src/linorobot2/linorobot2_gazebo
    touch COLCON_IGNORE

#### 2.3 Install linorobot2 package:
    
    cd <your_ws>
    rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent
    colcon build
    source install/setup.bash

* microxrcedds_agent dependency checks are skipped to prevent this [issue](https://github.com/micro-ROS/micro_ros_setup/issues/138) of finding its keys. This means that you have to always add `--skip-keys microxrcedds_agent` whenever you have to run `rosdep install` on the ROS2 workspace where you installed linorobot2.

## ENV Variables
### 1. Robot Type
Set LINOROBOT2_BASE env variable to the type of robot base that you want to use. This is not required if you're using a custom URDF. Available env variables are *2wd*, *4wd*, and *mecanum*. For example:

    echo "export LINOROBOT2_BASE=2wd" >> ~/.bashrc

### 2. Sensors
#### 2.1 Laser Sensor (Optional)
The launch files of the tested laser sensors have already been added in bringup.launch.py. You can enable one of these sensors by exporting the laser sensor you're using to `LINOROBOT2_LASER_SENSOR` env variable.

Tested Laser Sensors:
- `rplidar` - [RP LIDAR A1](https://www.slamtec.com/en/Lidar/A1)
- `ldlidar` - [LD06 LIDAR](https://www.inno-maker.com/product/lidar-ld06/)
- `realsense` - [Intel RealSense](https://www.intelrealsense.com/stereo-depth/) D435, D435i
- `astra` - [Orbec Astra](https://orbbec3d.com/product-astra-pro/)

For example:

    echo "export LINOROBOT2_LASER_SENSOR=rplidar" >> ~/.bashrc

If you export realsense to `LINOROBOT2_LASER_SENSOR`, the launch file will run [depthimage_to_laserscan](https://github.com/ros-perception/depthimage_to_laserscan) to convert the depth sensor's depth image to laser.

#### 2.2 Depth Sensor (Optional)
The Nav2 config file has been configured to support [Voxel Layer](https://navigation.ros.org/configuration/packages/costmap-plugins/voxel.html) for marking 3D obstacles in the Local Costmap using a depth sensor. To enable one of the tested depth sensor's launch file in bringup.launch.py, export the depth sensor you're using to `LINOROBOT2_DEPTH_SENSOR` env variable.

Tested sensors are:
- `realsense` - [Intel RealSense](https://www.intelrealsense.com/stereo-depth/) D435, D435i
- `astra` - [Orbec Astra](https://orbbec3d.com/product-astra-pro/)

For example:

    echo "export LINOROBOT2_DEPTH_SENSOR=realsense" >> ~/.bashrc

### 3. Save changes
Source your `~/.bashrc` to apply the changes you made:

    source ~/.bashrc

