#!/bin/bash

source /opt/ros/rosdistro/setup.bash
source /root/linorobot2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/.shm_off.xml

$@