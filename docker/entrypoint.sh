#!/usr/bin/env bash
set -e
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$WORKSPACE/install/setup.bash"


if [ "${VIRTUALGL_ENABLED}" = "true" ] && [ -x "$(command -v vglrun)" ]; then
    exec vglrun +v -d /dev/dri/card${GPU_ID:-0} "$@"
else
    exec "$@"
fi