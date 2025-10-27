#!/usr/bin/env bash
set -e
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/$USER/linorobot2_ws/install/setup.bash"
exec "$@"