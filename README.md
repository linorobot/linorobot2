## Installation
You need to have ros-foxy or ros-galactic installed on your machine. If you haven't installed ROS2 yet, you can use this [installer](https://github.com/linorobot/ros2me) script (works on x86 and ARM based dev boards ie. Raspberry Pi4/Nvidia Jetson Series).

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

### 2. Download linorobot2 and its dependencies:

#### 2.1 Download linorobot2:

    cd <your_ws> 
    git clone https://github.com/linorobot/linorobot2.git src/linorobot2

#### 2.2 Ignore Gazebo Packages on robot computer (optional)

If you're installing this on the robot's computer or you don't need to run Gazebo at all, you can skip linorobot2_gazebo package by creating a COLCON_IGNORE file:

    cd linorobot2/linorobot2_gazebo
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
Set LINOROBOT2_BASE env variable to the type of robot base that you want to use. Available env variables are *2wd*, *4wd*, and *mecanum*. For example:

    echo "export LINOROBOT2_BASE=2wd" >> ~/.bashrc

* Take note that this has no effect if you're using a custom URDF.

#### 1.1.b Laser Sensor (Optional)

There are pre-written launch files for tested sensors that are included in bringup.launch. You just have to define it as an env variable and you're good to go. Tested sensors are:
- rplidar
- ldlidar

For example:

    echo "export LINOROBOT2_LASER_SENSOR=rplidar" >> ~/.bashrc

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
### 1. Boot up your robot

#### 1.1a Using real robot:

    roslaunch linorobot2 bringup.launch.py

Optional parameter:
- serial_port - Your robot microcontroller's serial port. The default value is `/dev/ttyACM0` so remember to use this argument with the correct serial port otherwise. For example:

        roslaunch linorobot2 bringup.launch.py serial_port:=/dev/ttyACM1

#### 1.1b Using Gazebo:
    
    roslaunch linorobot2 gazebo.launch.py

### 2. Create a map

#### 2.1 Run the SLAM package:

    roslaunch linorobot2 slam.launch.py

Optional parameters:
- rviz - Set to true if you want to run RVIZ in parallel. Default value is false.
- sim - Set to true if you're running with Gazebo. Default value is false.

For example:

    roslaunch linorobot2 slam.launch.py rviz:=true sim:=true

#### 2.2 Move the robot to start mapping

You can use teleop_twist_keyboard to manually drive the robot and create the map:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Alternatively, you can also drive the robot autonomously by sending goal poses to the robot in rviz:

    ros2 launch nav2_bringup navigation_launch.py

- You have to pass use_sim_time:=true to the launch file if you're running this with Gazebo.


#### 2.3 Save the map

    cd linorobot2/linorobot2_navigation/maps
    ros2 run nav2_map_server map_saver_cli -f <map_name> --ros-args -p save_map_timeout:=10000

### 3. Autonomous Navigation

#### 3.1a Load the map you created:

Open linorobot2/linorobot2_navigation/launch/navigation.launch.py and change *MAP_NAME* to the name of the map you just created. Once done, build your workspace:
    
    cd <your_ws>
    colcon build

* Take note that you only have to do this when you need to change the map. 


#### 3.1b Run the navigation package:

    ros2 launch linorobot2_navigation navigation.launch.py

Optional parameters:
- rviz - Set to true if you want to run RVIZ in parallel. Default value is false.
- sim - Set to true if you're running with Gazebo. Default value is false.
- map - Path of <your_map.yaml> you want to use.

## Troubleshooting Guide

#### 1. The changes I made on a file is not taking effect on the package configuration/robot's behavior.
- You need to build your workspace every time you modify a file:

        cd <your_ws>
        colcon build
        #continue what you're doing...
