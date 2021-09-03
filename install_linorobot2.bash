#!/usr/bin/env bash

set -e

ROSDISTRO="$(rosversion -d)"
BASE=$1
MACHINE=$2
LASER_SENSOR=$3
DEPTH_SENSOR=$4
ARCH="$(uname -m)"
WORKSPACE="$HOME/linorobot2_ws"

if [[ "$ROSDISTRO" == "" ]]
    then
        echo "No ROS2 distro detected"
        echo "Try running $ source /opt/ros/<ros_distro>/setup.bash"
fi

if [ "$*" == "" ]
    then
        echo "No arguments provided"
        echo
        echo "Example: $ bash install_linorobot2.bash 2wd rplidar"
        echo "Example: $ bash install_linorobot2.bash 2wd rplidar realsense"
        echo "Example: $ bash install_linorobot2.bash 2wd - realsense"
        echo "Example: $ bash install_linorobot2.bash 2wd"

        echo
        exit 1
fi
        
if [[ "$BASE" != "2wd" && "$BASE" != "4wd" && "$BASE" != "mecanum" ]]
    then
        echo "Invalid linorobot base: $1"
        echo
        echo "Valid Options:"
        echo "2wd"
        echo "4wd"
        echo "mecanum"
        echo
        exit 1
fi

if [[ "$MACHINE" != "remote" && "$MACHINE" != "robot" && "$MACHINE" != "ci" ]]
    then
        echo "Invalid linorobot base: $1"
        echo
        echo "Valid Options:"
        echo "remote"
        echo "robot"
        echo
        exit 1
fi

if [[ "$LASER_SENSOR" != "rplidar" && "$LASER_SENSOR" != "ldlidar" && "$LASER_SENSOR" != "realsense" && "$LASER_SENSOR" != "-" && "$LASER_SENSOR" != "" ]]
    then
        echo "Invalid linorobot2 laser sensor: $LASER_SENSOR"
        echo
        echo "Valid Options:"
        echo "rplidar"
        echo "ldlidar"
        echo "realsense"
        echo "-"
        echo
        exit 1
fi

if [[ "$DEPTH_SENSOR" != "realsense" && "$DEPTH_SENSOR" != "" ]]
    then
        echo "Invalid linorobot2 depth sensor: $DEPTH_SENSOR"
        echo
        echo "Valid Options:"
        echo "realsense"
        echo
        exit 1
fi

if [[ "$MACHINE" != "ci" ]]
    then
        echo
        echo "You are installing linorobot2 on your $MACHINE computer."
        echo ""
        echo "===========SUMMARY============"
        echo "ROBOT TYPE   : $BASE"
        echo "LASER SENSOR : $LASER_SENSOR"
        echo "DEPTH SENSOR : $DEPTH_SENSOR"
        echo ""
        echo "This installer will edit your ~/.bashrc."
        echo "Create a linorobot2_ws on your $HOME directory."
        echo "Install linorobot2 ROS2 dependencies."
        echo "Install udev rules on /etc/udev/rules.d folder."
        echo -n "Enter [y] to continue. " 
        read reply
        if [[ "$reply" != "y" && "$reply" != "Y" ]]
            then
                echo "Exiting now."
                exit 1
        fi
fi

echo
echo "INSTALLING NOW...."
echo

sudo apt update

#### 1.1 Source your ROS2 distro and workspace
cd $HOME
mkdir -p $WORKSPACE/src
source /opt/ros/$ROS_DISTRO/setup.bash
cd $WORKSPACE
colcon build
source $WORKSPACE/install/setup.bash

#### 1.2 Download and install micro-ROS:
cd $WORKSPACE
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo apt install -y python3-vcstool
sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y
colcon build
source $WORKSPACE/install/setup.bash

#### 1.3 Setup micro-ROS agent:
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source $WORKSPACE/install/setup.bash

#### 1.4 Install LIDAR ROS2 drivers (if you're using one of the tested ones):
if [[ "$LASER_SENSOR" == "rplidar" ]]
    then
        sudo apt install -y ros-$ROS_DISTRO-rplidar-ros
        cd /tmp
        wget https://raw.githubusercontent.com/allenh1/rplidar_ros/ros2/scripts/rplidar.rules
        sudo cp rplidar.rules /etc/udev/rules.d/
        sudo service udev reload
        sudo service udev restart

elif [[ "$LASER_SENSOR" == "ldlidar" ]]
    then
        cd $WORKSPACE
        git clone https://github.com/linorobot/ldlidar src/ldlidar
        sudo cp src/ldlidar/ldlidar.rules /etc/udev/rules.d/
        sudo service udev reload
        sudo service udev restart

elif [[ "$LASER_SENSOR" == "realsense" ]]
    then
        sudo apt install -y ros-$ROS_DISTRO-realsense2-camera
fi

#### 1.5 Install depth sensor drivers (if you're using on of the tested ones):
if [[ "$DEPTH_SENSOR" == "realsense" ]]
    then
        sudo apt install -y ros-$ROS_DISTRO-realsense2-camera
fi

if [[ "$MACHINE" == "ci" ]]
    then
        cd $WORKSPACE
        git clone https://github.com/linorobot/ldlidar src/ldlidar
        sudo apt install -y ros-$ROS_DISTRO-rplidar-ros
        sudo apt install -y ros-$ROS_DISTRO-realsense2-camera
fi

#### 2.1 Download linorobot2:
cd $WORKSPACE
git clone https://github.com/linorobot/linorobot2 src/linorobot2

#### 2.2 Ignore Gazebo Packages on robot computer (optional)
if [[ "$MACHINE" == "robot" ]]
    then
        cd $WORKSPACE/src/linorobot2/linorobot2_gazebo
        touch COLCON_IGNORE
fi

#### 2.3 Install linorobot2 package:
cd $WORKSPACE
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent
colcon build
source $WORKSPACE/install/setup.bash

## ENV Variables
if [[ "$MACHINE" != "ci" ]]
    then
        ### 1. Robot Type
        echo "export LINOROBOT2_BASE=$BASE" >> ~/.bashrc
        ### 2. Sensors
        echo "export LINOROBOT2_LASER_SENSOR=$LASER_SENSOR" >> ~/.bashrc
        echo "export LINOROBOT2_DEPTH_SENSOR=$DEPTH_SENSOR" >> ~/.bashrc
        echo ""
        echo "Do you want to add sourcing of linorobot2_ws on your ~/.bashrc?"
        echo -n "Yes [y] or No [n]: " 
        read reply
        if [[ "$reply" == "y" || "$reply" == "Y" ]]
            then
                echo "source \$HOME/linorobot2_ws/install/setup.bash" >> ~/.bashrc
        else
            echo ""
            echo "Remember to do run $ source ~/linorobot2_ws/install/setup.bash every time you open a terminal."
        fi
fi

echo ""
echo "INSTALLATION DONE."