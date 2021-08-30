## Installation
You need to have ros-foxy or ros-galactic installed on your machines. If you haven't installed ROS2 yet, you can use this [installer](https://github.com/linorobot/ros2me) script (works on x86 and ARM based dev boards ie. Raspberry Pi4/Nvidia Jetson Series). Take note that [Installation](https://github.com/linorobot/linorobot2#installation) and [Setting Up](https://github.com/linorobot/linorobot2#setting-up) must be done on your robot computer (ie. Nvidia Jetson Board/Raspberry Pi) and host machine (for visualization and teleoperation).

### 1. Install micro-ROS and its dependencies

#### 1.1 Source your ROS2 distro and workspace
If it's your first time using ROS2 and haven't created your ROS2 workspace yet, you can check out [ROS2 Creating a Workspace](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html) tutorial.

    source /opt/ros/<your_ros_distro>/setup.bash
    cd <your_ws>
    colcon build
    source install/setup.bash

#### 1.2 Download and install micro-ROS:

    cd <your_ws>
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
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

#### 1.4 Install LIDAR ROS2 drivers (if you're using one of the tested ones):
RPLIDAR:

    sudo apt install ros-$ROS_DISTRO-rplidar-ros

LDLIDAR:

    cd <your_ws>
    git clone https://github.com/linorobot/ldlidar src/ldlidar

#### 1.5 Install depth sensor drivers (if you're using on of the tested ones):
Intel RealSense:

    sudo apt install ros-$ROS_DISTRO-realsense2-camera


### 2. Download linorobot2 and its dependencies:

#### 2.1 Download linorobot2:

    cd <your_ws> 
    git clone https://github.com/linorobot/linorobot2.git src/linorobot2

#### 2.2 Ignore Gazebo Packages on robot computer (optional)

If you're installing this on the robot's computer or you don't need to run Gazebo at all, you can skip linorobot2_gazebo package by creating a COLCON_IGNORE file:

    cd src/linorobot2/linorobot2_gazebo
    touch COLCON_IGNORE

#### 2.3 Install linorobot2 package:
    
    cd <your_ws>
    rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent
    colcon build
    source install/setup.bash

* microxrcedds_agent dependency checks are skipped to prevent this [issue](https://github.com/micro-ROS/micro_ros_setup/issues/138) of finding its keys.


## Setting Up
### 1. ENV Variables
#### 1.1.a Robot Type
Set LINOROBOT2_BASE env variable to the type of robot base that you want to use. This is not required if you're using a custom URDF. Available env variables are *2wd*, *4wd*, and *mecanum*. For example:

    echo "export LINOROBOT2_BASE=2wd" >> ~/.bashrc

#### 1.1.b Laser Sensor (Optional)
The launch files of the tested laser sensors have already been added in bringup.launch.py. You can enable one of these sensors by exporting the laser sensor you're using to `LINOROBOT2_LASER_SENSOR` env variable.

Tested Laser Sensors:
- `rplidar` - [RP LIDAR A1](https://www.slamtec.com/en/Lidar/A1)
- `ldlidar` - [LD06 LIDAR](https://www.inno-maker.com/product/lidar-ld06/)

For example:

    echo "export LINOROBOT2_LASER_SENSOR=rplidar" >> ~/.bashrc

#### 1.2.c Depth Sensor (Optional)
The Nav2 config file has been configured to support Voxel Layer for marking 3D obstacles in the Global Costmap using a depth sensor. To enable one of the tested depth sensor's launch file in bringup.launch.py, export the depth sensor you're using to `LINOROBOT2_DEPTH_SENSOR` env variable.

Tested sensors are:
- `realsense` - [Intel RealSense](https://www.intelrealsense.com/stereo-depth/) D435, D435i

For example:

    echo "export LINOROBOT2_DEPTH_SENSOR=realsense" >> ~/bashrc

#### 1.2 Source ~/.bashrc
Source your `~/.bashrc` to apply the changes you made:

    source ~/.bashrc

### 2.1. URDF
[linorobot2_description](https://github.com/linorobot/linorobot2/tree/master/linorobot2_description) package has parametized xacro files that can help you kickstart writing your URDF. Open <your_robot_type>.properties.urdf.xacro in [linorobot2_description/urdf](https://github.com/linorobot/linorobot2/tree/master/linorobot2_description/urdf) folder and change the values according to your robot's specification. Keep in mind that all pose definitions must be measured from the `base_link` (center of base) and wheel positions (ie `wheel_pos_x`) are referring to wheel 1.

Robot Orientation:

--------------FRONT--------------

WHEEL1  WHEEL2  (2WD)

WHEEL3  WHEEL4  (4WD)

--------------BACK--------------

Build your workspace once you're done:

    cd <your_workspace>
    colcon build

You can visualize the robot you built by running:

    ros2 launch linorobot2_description description.launch.py rviz:=true

If you already have an existing URDF, you can change the `urdf_path` in [description.launch.py](https://github.com/linorobot/linorobot2/blob/master/linorobot2_description/launch/description.launch.py) found in linorobot2_description/launch folder. Remember to build your workspace after editing the file.

## Quickstart
All commands below are to be run on your robot computer unless you're running a simulation or rviz2 for visualizing the robot. SLAM and Navigation launch files are the same for both real and simulated robots in Gazebo.

If you're unfamiliar how to access your robot computer remotely using [ssh](https://itsfoss.com/set-up-ssh-ubuntu/), connect your robot computer to a monitor and install openssh-server.

    sudo apt install openssh-server

Now you can easily access your robot computer using its [IP address](https://itsfoss.com/check-ip-address-ubuntu/) and run commands without a monitor. For example:

    ssh <your_user_name>@<robot_compueter_ip_address>

### 1. Boot up your robot

#### 1.1a Using a real robot:

    ros2 launch linorobot2_bringup bringup.launch.py

Optional parameters:
- **base_serial_port** - Your robot microcontroller's serial port. The default value is `/dev/ttyACM0` but remember to use this argument and change the default value to the correct serial port. For example:
    
    ```
    ros2 launch linorobot2_bringup bringup.launch.py base_serial_port:=/dev/ttyACM1
    ```

- **joy** - Set to true if you want to run the joystick node in the background. (Tested on Logitech F710).

#### 1.1b Using Gazebo:
    
    ros2 launch linorobot2_bringup gazebo.launch.py

Always remember to run linorobot2_bringup.launch.py on a separate terminal before creating a map or robot navigation.

### 2. Create a map

#### 2.1 Run [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox):


    ros2 launch linorobot2_navigation slam.launch.py

Optional parameters

For example:

    ros2 launch linorobot2_navigation slam.launch.py rviz:=true sim:=true

- **sim** - Set to true if you're running with Gazebo. Default value is false.
- **rviz** - Set to true if you want to run RVIZ in parallel. Default value is false. This won't work on headless setup but you can visualize the map created from your host computer by running:
   
    ```
    cd linorobot2/linorobot2_navigation/rviz
    rviz2 -d linorobot2_slam.rviz
    ```

#### 2.2 Move the robot to start mapping

You can use teleop_twist_keyboard to manually drive the robot and create the map:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Alternatively, you can also drive the robot autonomously by sending goal poses to the robot in rviz:

    ros2 launch nav2_bringup navigation_launch.py

- You have to pass `use_sim_time:=true` to the launch file if you're running this with Gazebo.


#### 2.3 Save the map

    cd linorobot2/linorobot2_navigation/maps
    ros2 run nav2_map_server map_saver_cli -f <map_name> --ros-args -p save_map_timeout:=10000

### 3. Autonomous Navigation

#### 3.1a Load the map you created:

Open linorobot2/linorobot2_navigation/launch/navigation.launch.py and change *MAP_NAME* to the name of the map you just created. Once done, build your workspace:
    
    cd <your_ws>
    colcon build

* You only have to do this when you need to change the map. 

#### 3.1b Run [Nav2](https://navigation.ros.org/tutorials/docs/navigation2_on_real_turtlebot3.html) package:

    ros2 launch linorobot2_navigation navigation.launch.py

Optional parameters:
- **sim** - Set to true if you're running with Gazebo. Default value is false.
- **map** - Path of <your_map.yaml> you want to use.
- **rviz** - Set to true if you want to run RVIZ in parallel. Default value is false. This won't work on headless setup but you can visualize the robot from your host computer by running:

    ```
    cd linorobot2/linorobot2_navigation/rviz
    rviz2 -d linorobot2_navigation.rviz
    ```

If you're new to ROS, you can checkout Nav2's [tutorial](https://navigation.ros.org/tutorials/docs/navigation2_on_real_turtlebot3.html#initialize-the-location-of-turtlebot-3) on how to initialize and send goal pose. 

## Troubleshooting Guide

#### 1. The changes I made on a file is not taking effect on the package configuration/robot's behavior.
- You need to build your workspace every time you modify a file:

    ```
    cd <your_ws>
    colcon build
    #continue what you're doing...
    ```

#### 2. [`slam_toolbox]: Message Filter dropping message: frame 'laser'`
- Try to up `transform_timeout` by 0.1 in linorobot2_navigation/config/slam.yaml until the warning is gone.
