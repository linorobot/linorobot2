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

function show_help {
    echo "Usage: $(basename $0) [OPTIONS]"
    echo
    echo "Install linorobot2 and its dependencies."
    echo
    echo "Options:"
    echo "  -b, --base <type>       Robot base type. Required for full robot install."
    echo "                          Valid values: 2wd, 4wd, mecanum"
    echo "  -l, --laser <sensor>    Laser sensor to install driver for."
    echo "                          Valid values: ydlidar, xv11, ld06, ld19, stl27l,"
    echo "                                        a1, a2, a3, c1, s1, s2, s3, ldlidar,"
    echo "                                        realsense, zed, zedm, zed2, zed2i,"
    echo "                                        oakd, oakdlite, oakdpro"
    echo "  -d, --depth <sensor>    Depth sensor to install driver for."
    echo "                          Valid values: realsense, zed, zedm, zed2, zed2i,"
    echo "                                        oakd, oakdlite, oakdpro"
    echo "  -w, --workspace <path>  Target workspace path. (default: \$HOME/linorobot2_ws)"
    echo "      --exclude-udev      Skip udev rule installation (e.g. for Docker builds)."
    echo "      --udev-only         Only install udev rules; skip all driver and workspace"
    echo "                          setup. --base is not required in this mode."
    echo "  -h, --help              Show this help message and exit."
    echo
    echo "Examples:"
    echo "  $(basename $0) --base 2wd --laser a1"
    echo "  $(basename $0) --base 4wd --laser realsense --depth realsense"
    echo "  $(basename $0) --laser ydlidar"
    echo "  $(basename $0) --laser ld06 --udev-only"
    echo "  $(basename $0) --base 2wd --laser a1 --exclude-udev"
}

EXCLUDE_UDEV=false
UDEV_ONLY=false

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --base|-b)      BASE="$2";         shift ;;
        --laser|-l)     LASER_SENSOR="$2"; shift ;;
        --depth|-d)     DEPTH_SENSOR="$2"; shift ;;
        --workspace|-w) WORKSPACE="$2";    shift ;;
        --exclude-udev) EXCLUDE_UDEV=true ;;
        --udev-only)    UDEV_ONLY=true ;;
        --help|-h)      show_help; exit 0 ;;
        *) echo "Unknown argument: $1"; echo; show_help; exit 1 ;;
    esac
    shift
done

if [ "$EXCLUDE_UDEV" = "true" ] && [ "$UDEV_ONLY" = "true" ]; then
    echo "Error: --exclude-udev and --udev-only are mutually exclusive."
    echo
    show_help
    exit 1
fi

if [ "$UDEV_ONLY" = "true" ]; then
    if [[ -z "$LASER_SENSOR" && -z "$DEPTH_SENSOR" ]]; then
        echo "Error: --udev-only requires at least --laser or --depth."
        echo
        show_help
        exit 1
    fi
elif [[ -z "$BASE" && -z "$LASER_SENSOR" && -z "$DEPTH_SENSOR" ]]; then
    echo "Error: at least one option is required."
    echo
    show_help
    exit 1
fi

set -e

ROSDISTRO="$(printenv ROS_DISTRO || true)"
ARCH="$(uname -m)"
WORKSPACE="${WORKSPACE:-$HOME/linorobot2_ws}"

ROBOT_TYPE_ARRAY=(2wd 4wd mecanum)
DEPTH_SENSOR_ARRAY=(realsense zed zedm zed2 zed2i oakd oakdlite oakdpro)
LASER_SENSOR_ARRAY=(ydlidar xv11 ld06 ld19 stl27l a1 a2 a3 c1 s1 s2 s3 ldlidar)
LASER_SENSOR_ARRAY+=(${DEPTH_SENSOR_ARRAY[@]})

####################################
# Sensor driver functions
# - Pure driver installation only; no udev operations.
# - To add a new sensor: define install_<sensor> here.
####################################

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
    colcon build
    source $WORKSPACE/install/setup.bash
}

function install_ldlidar_stl_ros2 {
    cd $WORKSPACE
    git clone https://github.com/hippo5329/ldlidar_stl_ros2.git src/ldlidar_stl_ros2
    colcon build
    source $WORKSPACE/install/setup.bash
}

function install_ld06    { install_ldlidar_stl_ros2; }
function install_ld19    { install_ldlidar_stl_ros2; }
function install_stl27l  { install_ldlidar_stl_ros2; }

function install_ldlidar {
    cd $WORKSPACE
    git clone https://github.com/linorobot/ldlidar src/ldlidar
    colcon build
    source $WORKSPACE/install/setup.bash
}

function install_sllidar_ros2 {
    cd $WORKSPACE
    git clone https://github.com/Slamtec/sllidar_ros2.git
    colcon build
    source $WORKSPACE/install/setup.bash
}

function install_a1  { install_sllidar_ros2; }
function install_a2  { install_sllidar_ros2; }
function install_a3  { install_sllidar_ros2; }
function install_c1  { install_sllidar_ros2; }
function install_s1  { install_sllidar_ros2; }
function install_s2  { install_sllidar_ros2; }
function install_s3  { install_sllidar_ros2; }

function install_realsense {
    sudo apt install -y ros-$ROS_DISTRO-realsense2-camera
}

function install_astra {
    cd $WORKSPACE
    sudo apt install -y libuvc-dev libopenni2-dev
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
    colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release
    source $WORKSPACE/install/setup.bash
    source ~/.bashrc
}

function install_zedm  { install_zed; }
function install_zed2  { install_zed; }
function install_zed2i { install_zed; }

function install_oakd {
    sudo apt install ros-$ROS_DISTRO-depthai-ros
}

function install_oakdlite { install_oakd; }
function install_oakdpro  { install_oakd; }

####################################
# Sensor udev functions
# - Pure udev rule installation only; no driver operations.
# - Only defined for sensors that have udev rules.
# - Sensors without a udev_<sensor> function have no udev rules.
# - To add udev rules for a new sensor: define udev_<sensor> here.
####################################

function udev_ydlidar {
    sudo echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar.rules
    sudo echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-V2.rules
    sudo echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-2303.rules
}

function udev_ldlidar_stl_ros2 {
    cd /tmp
    wget https://raw.githubusercontent.com/linorobot/ldlidar/ros2/ldlidar.rules
    sudo cp ldlidar.rules /etc/udev/rules.d
}

function udev_ld06   { udev_ldlidar_stl_ros2; }
function udev_ld19   { udev_ldlidar_stl_ros2; }
function udev_stl27l { udev_ldlidar_stl_ros2; }

function udev_sllidar_ros2 {
    # Clone repo if not already present (rules file lives inside the repo)
    if [ ! -d "$WORKSPACE/sllidar_ros2" ]; then
        cd $WORKSPACE
        git clone https://github.com/Slamtec/sllidar_ros2.git
    fi
    sudo cp $WORKSPACE/sllidar_ros2/scripts/rplidar.rules /etc/udev/rules.d
}

function udev_a1  { udev_sllidar_ros2; }
function udev_a2  { udev_sllidar_ros2; }
function udev_a3  { udev_sllidar_ros2; }
function udev_c1  { udev_sllidar_ros2; }
function udev_s1  { udev_sllidar_ros2; }
function udev_s2  { udev_sllidar_ros2; }
function udev_s3  { udev_sllidar_ros2; }

function udev_realsense {
    cd /tmp
    wget https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
    sudo cp 99-realsense-libusb.rules /etc/udev/rules.d
}

function udev_astra {
    # Clone repo if not already present (rules file lives inside the repo)
    if [ ! -d "$WORKSPACE/src/ros_astra_camera" ]; then
        cd $WORKSPACE
        git clone https://github.com/linorobot/ros_astra_camera src/ros_astra_camera
    fi
    sudo cp $WORKSPACE/src/ros_astra_camera/56-orbbec-usb.rules /etc/udev/rules.d/
}

function udev_oakd {
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger
}

function udev_oakdlite { udev_oakd; }
function udev_oakdpro  { udev_oakd; }

# No udev rules: xv11, ldlidar, zed, zedm, zed2, zed2i, cuda_jetson

####################################
# Sensor install dispatcher
# Centralises all --exclude-udev / --udev-only logic in one place.
# Sensor functions above stay clean and flag-free.
####################################

function run_install {
    local sensor=$1

    if [ "$UDEV_ONLY" = "true" ]; then
        if declare -f "udev_${sensor}" > /dev/null 2>&1; then
            udev_${sensor}
        else
            echo "No udev operation for ${sensor}"
        fi
        return
    fi

    install_${sensor}

    if [ "$EXCLUDE_UDEV" != "true" ] && declare -f "udev_${sensor}" > /dev/null 2>&1; then
        udev_${sensor}
    fi
}

####################################
# Core install functions
####################################

function setup_workspace {
    mkdir -p $WORKSPACE/src
    source /opt/ros/$ROS_DISTRO/setup.bash
    cd $WORKSPACE
    colcon build
    source $WORKSPACE/install/setup.bash
}

function install_microros {
    cd $WORKSPACE
    if [ ! -d src/micro_ros_setup ]
        then
            git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
    fi
    sudo apt install -y python3-vcstool build-essential
    sudo apt update && rosdep update
    rosdep init || echo "rosdep already initialized"
    rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent --skip-keys micro_ros_agent
    colcon build
    source $WORKSPACE/install/setup.bash
}

function setup_microros_agent {
    cd $WORKSPACE
    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    source $WORKSPACE/install/setup.bash
}

function install_linorobot2_pkg {
    cd $WORKSPACE
    if [ ! -d src/linorobot2 ]
        then
            git clone -b $ROS_DISTRO https://github.com/linorobot/linorobot2 src/linorobot2
    fi
    cd $WORKSPACE/src/linorobot2/linorobot2_gazebo
    touch COLCON_IGNORE
    cd $WORKSPACE
    rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent
    colcon build
    source $WORKSPACE/install/setup.bash
}

####################################
# Validation
####################################

if [ "$UDEV_ONLY" != "true" ] && [[ "$ROSDISTRO" == "" || "$ROSDISTRO" == "<unknown>" ]]
    then
        echo "No ROS2 distro detected"
        echo "Try running $ source /opt/ros/<ros_distro>/setup.bash and try again."
        exit 1
fi

if [[ -n "$BASE" ]] && !(printf '%s\n' "${ROBOT_TYPE_ARRAY[@]}" | grep -xq "$BASE")
    then
        echo "Invalid linorobot base: $BASE"
        echo
        echo "Valid Options:"
        for key in "${!ROBOT_TYPE_ARRAY[@]}"; do echo "${ROBOT_TYPE_ARRAY[$key]}"; done
        echo
        exit 1
fi

if [[ -n "$LASER_SENSOR" ]] && !(printf '%s\n' "${LASER_SENSOR_ARRAY[@]}" | grep -xq "$LASER_SENSOR")
    then
        echo "Invalid linorobot2 laser sensor: $LASER_SENSOR"
        echo
        echo "Valid Options:"
        for key in "${!LASER_SENSOR_ARRAY[@]}"; do echo "${LASER_SENSOR_ARRAY[$key]}"; done
        echo
        exit 1
fi

if [[ -n "$DEPTH_SENSOR" ]] && !(printf '%s\n' "${DEPTH_SENSOR_ARRAY[@]}" | grep -xq "$DEPTH_SENSOR")
    then
        echo "Invalid linorobot2 depth sensor: $DEPTH_SENSOR"
        echo
        echo "Valid Options:"
        for key in "${!DEPTH_SENSOR_ARRAY[@]}"; do echo "${DEPTH_SENSOR_ARRAY[$key]}"; done
        echo
        exit 1
fi

####################################
# Summary
####################################

echo
if [ "$UDEV_ONLY" = "true" ]; then
    echo "Installing udev rules only."
    echo
    echo "===========SUMMARY============"
    echo "LASER SENSOR : $LASER_SENSOR"
    echo "DEPTH SENSOR : $DEPTH_SENSOR"
elif [[ -n "$BASE" ]]; then
    echo "Installing linorobot2 on robot computer."
    echo
    echo "===========SUMMARY============"
    echo "ROBOT TYPE   : $BASE"
    echo "LASER SENSOR : $LASER_SENSOR"
    echo "DEPTH SENSOR : $DEPTH_SENSOR"
    echo "EXCLUDE UDEV : $EXCLUDE_UDEV"
else
    echo "Installing sensor drivers."
    echo
    echo "===========SUMMARY============"
    echo "LASER SENSOR : $LASER_SENSOR"
    echo "DEPTH SENSOR : $DEPTH_SENSOR"
    echo "EXCLUDE UDEV : $EXCLUDE_UDEV"
fi
echo

echo
echo "INSTALLING NOW...."
echo

####################################
# Installation
####################################

if [ "$UDEV_ONLY" != "true" ]; then
    if [[ -n "$BASE" ]]
        then
            setup_workspace
    else
            mkdir -p $WORKSPACE/src
    fi
fi

#### Sensor install (driver + udev, or just udev, depending on flags)
if [[ -n "$LASER_SENSOR" ]] && (printf '%s\n' "${LASER_SENSOR_ARRAY[@]}" | grep -xq "$LASER_SENSOR")
    then
        run_install $LASER_SENSOR
fi

if [[ -n "$DEPTH_SENSOR" ]] && (printf '%s\n' "${DEPTH_SENSOR_ARRAY[@]}" | grep -xq "$DEPTH_SENSOR")
    then
        run_install $DEPTH_SENSOR
fi

#### Full install (robot computer only)
if [[ -n "$BASE" ]] && [ "$UDEV_ONLY" != "true" ]
    then
        install_microros
        setup_microros_agent
        install_linorobot2_pkg

        ### ENV Variables
        echo "export LINOROBOT2_BASE=$BASE" >> ~/.bashrc
        if [[ -n "$LASER_SENSOR" ]]
            then
                echo "export LINOROBOT2_LASER_SENSOR=$LASER_SENSOR" >> ~/.bashrc
        fi
        if [[ -n "$DEPTH_SENSOR" ]]
            then
                echo "export LINOROBOT2_DEPTH_SENSOR=$DEPTH_SENSOR" >> ~/.bashrc
        fi
        echo "source \$HOME/linorobot2_ws/install/setup.bash" >> ~/.bashrc
fi

echo
echo "INSTALLATION DONE."
echo
if [[ -n "$BASE" ]] && [ "$UDEV_ONLY" != "true" ]
    then
        echo "Restart your robot computer now."
fi
