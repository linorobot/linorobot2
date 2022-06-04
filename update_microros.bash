#!/usr/bin/env bash

set -e 

cd ../uros/micro-ROS-Agent/
git pull
cd ../../../
. install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
. install/setup.bash

