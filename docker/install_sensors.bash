#!/bin/bash
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

BASE=$1
LASER_SENSOR=$2
DEPTH_SENSOR=$3
ARCH="$(uname -m)"
ROSDISTRO=$4

WORKSPACE="/root/linorobot2_ws"

ROBOT_TYPE_ARRAY=(2wd 4wd mecanum)
DEPTH_SENSOR_ARRAY=(realsense zed zedm zed2 zed2i oakd oakdlite oakdpro)
LASER_SENSOR_ARRAY=(rplidar ldlidar ydlidar xv11)
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

function install_rplidar {
    sudo apt-get install -y ros-$ROS_DISTRO-rplidar-ros
    cd /tmp
    wget https://raw.githubusercontent.com/allenh1/rplidar_ros/ros2/scripts/rplidar.rules
}

function install_ldlidar {
    cd $WORKSPACE
    git clone https://github.com/linorobot/ldlidar src/ldlidar
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
    colcon build --symlink-install
    source $WORKSPACE/install/setup.bash
}

function install_realsense {
    sudo apt-get install -y ros-$ROS_DISTRO-realsense2-camera
    cd /tmp
    wget https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
}

function install_astra {
    cd $WORKSPACE
    sudo apt-get install -y libuvc-dev libopenni2-dev
    git clone https://github.com/linorobot/ros_astra_camera src/ros_astra_camera
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
    sudo apt-get install ros-$ROS_DISTRO-depthai-ros
}

function install_oakdlite {
    install_oakd
}

function install_oakdpro {
    install_oakd
}

echo
echo "INSTALLING SENSORS NOW ..."
echo

source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt-get update
if (printf '%s\n' "${LASER_SENSOR_ARRAY[@]}" | grep -xq $LASER_SENSOR)
    then
        install_$LASER_SENSOR
fi

if (printf '%s\n' "${DEPTH_SENSOR_ARRAY[@]}" | grep -xq $DEPTH_SENSOR)
    then
        install_$DEPTH_SENSOR
fi

