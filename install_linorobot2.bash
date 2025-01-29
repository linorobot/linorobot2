#!/usr/bin/env bash
# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -e

ROSDISTRO="$(printenv ROS_DISTRO)"
BASE=$1
LASER_SENSOR=$2
DEPTH_SENSOR=$3
ARCH="$(uname -m)"
WORKSPACE="$HOME/linorobot2_ws"

ROBOT_TYPE_ARRAY=(2wd 4wd mecanum)
DEPTH_SENSOR_ARRAY=(realsense zed zedm zed2 zed2i oakd oakdlite oakdpro)
LASER_SENSOR_ARRAY=(ydlidar xv11 ld06 ld19 stl27l a1 a2 a3 c1 s1 s2 s3)
LASER_SENSOR_ARRAY+=(${DEPTH_SENSOR_ARRAY[@]})

if [ -z "$LASER_SENSOR" ]
    then
        LASER_SENSOR=""
fi

if [ -z "$DEPTH_SENSOR" ]
    then
        DEPTH_SENSOR=""
fi

function install_cuda_jetson {
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/sbsa/cuda-ubuntu2004.pin
    sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
    wget http://developer.download.nvidia.com/compute/cuda/11.4.2/local_installers/cuda-repo-ubuntu2004-11-4-local_11.4.2-470.57.02-1_arm64.deb
    sudo dpkg -i cuda-repo-ubuntu2004-11-4-local_11.4.2-470.57.02-1_arm64.deb
    sudo apt-key add /var/cuda-repo-ubuntu2004-11-4-local/7fa2af80.pub #verify this
    sudo apt-get update
    sudo apt-get -y install cuda
    # Errors were encountered while processing:
    #  /tmp/apt-dpkg-install-TvUCLd/14-libnvidia-compute-470_470.57.02-0ubuntu1_arm64.deb
    #  /tmp/apt-dpkg-install-TvUCLd/18-libnvidia-gl-470_470.57.02-0ubuntu1_arm64.deb
    # E: Sub-process /usr/bin/dpkg returned an error code (1)
}

function install_xv11 {
    cd $WORKSPACE
    git clone https://github.com/mjstn/xv_11_driver src/xv_11_driver
    colcon build
    source $WORKSPACE/install/setup.bash
}

function install_ydlidar {
    cd /tmp
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git
    mkdir YDLidar-SDK/build
    cd YDLidar-SDK/build
    cmake ..
    make
    sudo make install
    cd $WORKSPACE
    git clone https://github.com/YDLIDAR/ydlidar_ros2_driver src/ydlidar_ros2_driver
    chmod 0777 src/ydlidar_ros2_driver/startup/*
    sudo echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar.rules
    sudo echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-V2.rules
    sudo echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-2303.rules
    colcon build --symlink-install
    source $WORKSPACE/install/setup.bash
}

function install_ldlidar_stl_ros2 {
    cd $WORKSPACE
    git clone https://github.com/hippo5329/ldlidar_stl_ros2.git src/ldlidar_stl_ros2
    colcon build
    source $WORKSPACE/install/setup.bash
    cd /tmp
    wget https://raw.githubusercontent.com/linorobot/ldlidar/ros2/ldlidar.rules
    sudo cp ldlidar.rules /etc/udev/rules.d
}

function install_ld06 {
    install_ldlidar_stl_ros2
}

function install_ld19 {
    install_ldlidar_stl_ros2
}

function install_stl27l {
    install_ldlidar_stl_ros2
}

function install_sllidar_ros2 {
    cd $WORKSPACE
    git clone https://github.com/Slamtec/sllidar_ros2.git
    colcon build
    source $WORKSPACE/install/setup.bash
    sudo cp sllidar_ros2/scripts/rplidar.rules /etc/udev/rules.d
}

function install_a1 {
    install_sllidar_ros2
}

function install_a2 {
    install_sllidar_ros2
}

function install_a3 {
    install_sllidar_ros2
}

function install_c1 {
    install_sllidar_ros2
}

function install_s1 {
    install_sllidar_ros2
}

function install_s2 {
    install_sllidar_ros2
}

function install_s3 {
    install_sllidar_ros2
}

function install_realsense {
    sudo apt install -y ros-$ROS_DISTRO-realsense2-camera
    cd /tmp
    wget https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
    sudo cp 99-realsense-libusb.rules /etc/udev/rules.d
}

function install_astra {
    cd $WORKSPACE
    sudo apt install -y libuvc-dev libopenni2-dev
    git clone https://github.com/linorobot/ros_astra_camera src/ros_astra_camera
    sudo cp src/ros_astra_camera/56-orbbec-usb.rules /etc/udev/rules.d/
    colcon build
    source $WORKSPACE/install/setup.bash
}

function install_zed {
    cd /tmp
    if [[ -f /etc/nv_tegra_release ]]
        then
            #TODO ADD CUDA INSTALLATION HERE
            wget https://download.stereolabs.com/zedsdk/3.5/jp45/jetsons -O zed_sdk
    elif lspci | grep VGA | grep -o NVIDIA
        then
            wget https://download.stereolabs.com/zedsdk/3.5/cu111/ubuntu20 -O zed_sdk
    else
        echo "Linux Machine not supported by Zed Camera"
        exit 1
    fi
    
    chmod +x zed_sdk
    ./zed_sdk -- silent
    cd $WORKSPACE
    
    git clone https://github.com/stereolabs/zed-ros2-wrapper src/zed-ros2-wrapper
    git clone https://github.com/ros-perception/image_common -b $ROS_DISTRO src/image_common #https://github.com/stereolabs/zed-ros2-wrapper#image-transport-and-topic-subscriptions
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
    # colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --cmake-args=-DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.4
    source $WORKSPACE/install/setup.bash
    source ~/.bashrc
}

function install_zedm {
    install_zed
}

function install_zed2 {
    install_zed
}

function install_zed2i {
    install_zed
}

function install_oakd {
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger
    sudo apt install ros-$ROS_DISTRO-depthai-ros
}

function install_oakdlite {
    install_oakd
}

function install_oakdpro {
    install_oakd
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
        echo "Example: $ bash install_linorobot2.bash 2wd a1"
        echo "Example: $ bash install_linorobot2.bash 2wd a1 realsense"
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

#### 1.2/1.3 Install LIDAR/Depth Sensor ROS2 drivers:
if (printf '%s\n' "${LASER_SENSOR_ARRAY[@]}" | grep -xq $LASER_SENSOR)
    then
        install_$LASER_SENSOR
fi

if (printf '%s\n' "${DEPTH_SENSOR_ARRAY[@]}" | grep -xq $DEPTH_SENSOR)
    then
        install_$DEPTH_SENSOR
fi

if [[ "$BASE" == "ci" ]]
    then
        for key in "${!LASER_SENSOR_ARRAY[@]}"; do  install_${LASER_SENSOR_ARRAY[$key]}; done
fi

#### 1.4 Download and install micro-ROS:
cd $WORKSPACE
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo apt install -y python3-vcstool build-essential
sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y
colcon build
source $WORKSPACE/install/setup.bash

#### 1.5 Setup micro-ROS agent:
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source $WORKSPACE/install/setup.bash

#### 2.1 Download linorobot2:
cd $WORKSPACE
git clone -b $ROS_DISTRO https://github.com/linorobot/linorobot2 src/linorobot2

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
        if [[ "$LASER_SENSOR" != "-" ||  "$LASER_SENSOR" != "" ]]
            then
                echo "export LINOROBOT2_LASER_SENSOR=$LASER_SENSOR" >> ~/.bashrc
        fi

        if [[ "$DEPTH_SENSOR" != "-" ||  "$DEPTH_SENSOR" != "" ]]
            then
            echo "export LINOROBOT2_DEPTH_SENSOR=$DEPTH_SENSOR" >> ~/.bashrc
        fi
        echo
        echo "Do you want to add sourcing of linorobot2_ws on your ~/.bashrc?"
        echo -n "Yes [y] or No [n]: " 
        read reply
        if [[ "$reply" == "y" || "$reply" == "Y" ]]
            then
                echo "source \$HOME/linorobot2_ws/install/setup.bash" >> ~/.bashrc
        else
            echo
            echo "Remember to run $ source ~/linorobot2_ws/install/setup.bash every time you open a terminal."
        fi
fi

echo
echo "INSTALLATION DONE."
echo
echo "Restart your robot computer now."
