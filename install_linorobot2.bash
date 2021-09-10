#!/usr/bin/env bash

set -e

ROSDISTRO="$(rosversion -d)"
BASE=$1
LASER_SENSOR=$2
DEPTH_SENSOR=$3
ARCH="$(uname -m)"
WORKSPACE="$HOME/test_ws"

ROBOT_TYPE_ARRAY=(2wd 4wd mecanum)
DEPTH_SENSOR_ARRAY=(realsense)
LASER_SENSOR_ARRAY=(rplidar ldlidar ydlidar)
LASER_SENSOR_ARRAY+=(${DEPTH_SENSOR_ARRAY[@]})

function install_rplidar {
    sudo apt install -y ros-$ROS_DISTRO-rplidar-ros
    cd /tmp
    wget https://raw.githubusercontent.com/allenh1/rplidar_ros/ros2/scripts/rplidar.rules
    sudo cp rplidar.rules /etc/udev/rules.d/
}

function install_ldlidar {
    cd $WORKSPACE
    git clone https://github.com/linorobot/ldlidar src/ldlidar
    sudo cp src/ldlidar/ldlidar.rules /etc/udev/rules.d/
}

function install_ydlidar {
    cd /tmp
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git
    cd YDLidar-SDK/build
    cmake ..
    make
    sudo make install
    cd $WORKSPACE
    git clone https://github.com/YDLIDAR/ydlidar_ros2_driver src/ydlidar_ros2_driver
    chmod 0777 src/ydlidar_ros2_driver/startup/*
    sudo sh src/ydlidar_ros2_driver/startup/initenv.sh
}

function install_realsense {
    sudo apt install -y ros-$ROS_DISTRO-realsense2-camera
}

function install_astra {
    cd $WORKSPACE
    sudo apt install -y libuvc-dev libopenni2-dev
    git clone https://github.com/linorobot/ros_astra_camera src/ros_astra_camera
    sudo cp src/ros_astra_camera/56-orbbec-usb.rules /etc/udev/rules.d/
}

if [[ "$ROSDISTRO" == "" || "$ROSDISTRO" == "<unknown>" ]]
    then
        echo "No ROS2 distro detected"
        echo "Try running $ source /opt/ros/<ros_distro>/setup.bash and try again."
        exit 1
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
        
if [[ "$BASE" != "ci" ]] && !(printf '%s\n' "${ROBOT_TYPE_ARRAY[@]}" | grep -xq $BASE)
    then
        echo "Invalid linorobot base: $1"
        echo
        echo "Valid Options:"
        for key in "${!ROBOT_TYPE_ARRAY[@]}"; do echo "${ROBOT_TYPE_ARRAY[$key]}"; done
        echo
        exit 1
fi

if [[ "$BASE" != "ci" && "$LASER_SENSOR" != "" && "$LASER_SENSOR" != "-" ]] && !(printf '%s\n' "${LASER_SENSOR_ARRAY[@]}" | grep -xq $LASER_SENSOR)
    then
        echo "Invalid linorobot2 laser sensor: $LASER_SENSOR"
        echo
        echo "Valid Options:"
        for key in "${!LASER_SENSOR_ARRAY[@]}"; do echo "${LASER_SENSOR_ARRAY[$key]}"; done
        echo
        exit 1
fi

if [[ "$BASE" != "ci" && "$DEPTH_SENSOR" != "" ]] && !(printf '%s\n' "${DEPTH_SENSOR_ARRAY[@]}" | grep -xq $DEPTH_SENSOR)
    then
        echo "Invalid linorobot2 depth sensor: $DEPTH_SENSOR"
        echo
        echo "Valid Options:"
        for key in "${!DEPTH_SENSOR_ARRAY[@]}"; do echo "${DEPTH_SENSOR_ARRAY[$key]}"; done
        echo
        exit 1
fi

if [[ "$BASE" != "ci" ]]
    then
        echo
        echo "You are installing linorobot2 on your robot computer."
        echo
        echo "===========SUMMARY============"
        echo "ROBOT TYPE   : $BASE"
        echo "LASER SENSOR : $LASER_SENSOR"
        echo "DEPTH SENSOR : $DEPTH_SENSOR"
        echo
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
sudo apt install -y python3-vcstool build-essential
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
        install_rplidar

elif [[ "$LASER_SENSOR" == "ldlidar" ]]
    then
        install_ldlidar

elif [[ "$LASER_SENSOR" == "ydlidar" ]]
    then
        install_ydlidar

elif [[ "$LASER_SENSOR" == "realsense" ]]
    then
        install_realsense

elif [[ "$LASER_SENSOR" == "astra" ]]
    then 
        install_astra
fi

#### 1.5 Install depth sensor drivers (if you're using on of the tested ones):
if [[ "$DEPTH_SENSOR" == "realsense" ]]
    then
        install_realsense

elif [[ "$DEPTH_SENSOR" == "astra" ]]
    then
        install_astra
fi

if [[ "$BASE" == "ci" ]]
    then
        install_rplidar
        install_ldlidar
        install_ydlidar
        install_realsense
        install_astra
fi

#### 2.1 Download linorobot2:
cd $WORKSPACE
git clone https://github.com/linorobot/linorobot2 src/linorobot2

#### 2.2 Ignore Gazebo Packages on robot computer (optional)
cd $WORKSPACE/src/linorobot2/linorobot2_gazebo
touch COLCON_IGNORE

#### 2.3 Install linorobot2 package:
cd $WORKSPACE
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent
colcon build
source $WORKSPACE/install/setup.bash

## ENV Variables
if [[ "$BASE" != "ci" ]]
    then
        ### 1. Robot Type
        echo "export LINOROBOT2_BASE=$BASE" >> ~/.bashrc
        ### 2. Sensors
        echo "export LINOROBOT2_LASER_SENSOR=$LASER_SENSOR" >> ~/.bashrc
        echo "export LINOROBOT2_DEPTH_SENSOR=$DEPTH_SENSOR" >> ~/.bashrc
        echo
        echo "Do you want to add sourcing of linorobot2_ws on your ~/.bashrc?"
        echo -n "Yes [y] or No [n]: " 
        read reply
        if [[ "$reply" == "y" || "$reply" == "Y" ]]
            then
                echo "source \$HOME/linorobot2_ws/install/setup.bash" >> ~/.bashrc
        else
            echo
            echo "Remember to do run $ source ~/linorobot2_ws/install/setup.bash every time you open a terminal."
        fi
fi

echo
echo "INSTALLATION DONE."
echo
echo "Restart your robot computer now."
